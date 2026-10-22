// Pour ArduBot V2 + BNO055 IMU
// VERSION V24 SOFT GUARD + REJET CMD_VEL HORS PLAGE - 2026-09
// IMPORTANT : la regulation PI validee est CONSERVEE STRICTEMENT :
//   kp_left=kp_right=300, ki_left=ki_right=6000, ITERM_LIMIT=0.30
// Aucune securite latchee/hold/IMU n'est utilisee.
// Une CMD_VEL hors plage est rejetee : STOP immediat + integrales PI remises a zero.
// Montage vertical : face TOP dirigee vers la gauche du robot.
// Changement de repere applique a TOUS les vecteurs (gyro, acceleration, magnetometre) :
//   X_robot =  Y_capteur
//   Y_robot = -Z_capteur
//   Z_robot = -X_capteur
// Le quaternion est transforme avec la rotation inverse correspondante.
//
// Contrat d'unites de la liaison serie vers le Raspberry Pi :
//   CMD_VEL,left_mps,right_mps              : m/s
//   ENC,left_ticks,right_ticks              : ticks (compteurs entiers)
//   IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az      : quaternion sans unite, gyro rad/s,
//                                             acceleration lineaire m/s^2
//   MAG,mx,my,mz                            : microteslas (uT), repere robot
//   CAL,sys,gyro,accel,mag                  : niveaux entiers 0..3
//   DBG,targetL,targetR,measL,measR,...     : vitesses m/s, PWM [-255;255], ticks
//
// IMPORTANT : Adafruit_BNO055::getVector(VECTOR_GYROSCOPE) renvoie des degres/s.
// Le code convertit explicitement en rad/s avant l'envoi, comme exige par sensor_msgs/Imu.
// Adafruit_BNO055::getVector(VECTOR_MAGNETOMETER) renvoie des microteslas (uT).
// La conversion uT -> tesla est effectuee cote Raspberry avant sensor_msgs/MagneticField.
//
// Librairies Arduino a installer :
//   Adafruit BNO055
//   Adafruit Unified Sensor

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <microM.h>

// ========= CONFIG ROBOT =========

static const float WHEEL_DIAMETER_M = 0.030f;
static const float WHEEL_RADIUS_M   = WHEEL_DIAMETER_M * 0.5f;

// 12 ticks moteur * reduction reelle 29:1
static const long TICKS_PER_WHEEL_REV = 12L * 29L;  // 348 ticks/tr roue

static const unsigned long CONTROL_PERIOD_US = 10000;   // 100 Hz
static const unsigned long ODOM_PERIOD_MS    = 100;     // 10 Hz
static const unsigned long IMU_PERIOD_MS     = 50;      // 20 Hz
static const unsigned long CAL_PERIOD_MS     = 500;     // 2 Hz
static const unsigned long CMD_TIMEOUT_MS    = 500;     // sécurité

// ========= SECURITE ENCODEURS / COMMANDES =========
//
// Le code historique utilisait attachInterrupt(0/1), correspondant aux broches
// D2/D3 sur les cartes AVR usuelles (UNO/Mega). On les nomme explicitement.
static const uint8_t ENCODER_LEFT_PIN  = 2;
static const uint8_t ENCODER_RIGHT_PIN = 3;

// Active une polarisation interne pour eviter une entree flottante en cas de
// connecteur/cable momentanement ouvert. Mettre false uniquement si le montage
// impose explicitement une autre polarisation externe.
static const bool ENCODER_USE_INTERNAL_PULLUP = true;

// Filtre anti-impulsions dynamique par roue.
//
// Le seuil est calcule a partir de la consigne de CHAQUE roue :
//   v_allowed = clamp(1.50 * abs(target_mps) + 0.05, 0.20, 0.65)
//   min_edge_us = distance_par_tick / v_allowed
//
// Avec roue 30 mm et 360 ticks/tr :
//   target=0.10 m/s -> min_edge ~1354 us
//   target=0.20 m/s -> min_edge ~ 774 us
//   target=0.30 m/s -> min_edge ~ 542 us
//   target=0.40/0.50 m/s -> min_edge ~417 us
//
// Le calcul flottant est effectue HORS ISR. Les ISR ne font qu'une
// comparaison avec un seuil deja calcule.
static const float ENCODER_FILTER_MIN_ALLOWED_MPS = 0.20f;
static const float ENCODER_FILTER_GAIN = 1.50f;
static const float ENCODER_FILTER_MARGIN_MPS = 0.05f;
static const float ENCODER_FILTER_MAX_ALLOWED_MPS = 0.65f;

volatile unsigned long encoder_min_edge_left_us  = 1354UL;
volatile unsigned long encoder_min_edge_right_us = 1354UL;

// Plausibilite dynamique des encodeurs.
// Limite absolue volontairement tres au-dessus des vitesses normales du robot.
static const float ENCODER_ABSOLUTE_MAX_MPS = 1.00f;
static const float ENCODER_TARGET_MARGIN_MPS = 0.45f;
static const float ENCODER_ZERO_TARGET_ALLOW_MPS = 0.20f;
static const long  ENCODER_DELTA_MARGIN_TICKS = 4L;

// ========= V23 : SOFT GUARD NON BLOQUANT =========
//
// Principe : un front encodeur suspect ne doit JAMAIS provoquer un arret durable.
// On filtre la reaction du PI au lieu de couper les moteurs.
//
// 1) Si >=2 fronts ont ete rejetes sur une roue pendant un cycle de controle :
//    - l'integrale de cette roue n'augmente pas ;
//    - le PWM de cette roue ne peut pas augmenter pendant ce cycle.
//
// 2) Si la vitesse mesuree depasse la consigne de plus de 0.06 m/s :
//    - meme comportement : pas d'augmentation de l'integrale/PWM.
//
// 3) Une mesure encodeur totalement impossible est simplement ignoree pour
//    CE cycle. La consigne reste active et le PWM ne peut pas augmenter.
//
// 4) La montee de PWM est limitee a +25 par cycle de 10 ms.
//    Toute baisse de PWM reste immediate.
//
// 5) Le PI n'a jamais le droit d'inverser une roue contre le sens demande.
//    Une inversion reste parfaitement autorisee si la CMD_VEL elle-meme change
//    de signe (Nav2, rotation sur place, marche arriere, etc.).
//
// Aucun de ces mecanismes n'utilise l'IMU : la calibration IMU ne peut donc
// pas provoquer de blocage moteur.
static const unsigned long SOFT_GUARD_DIAG_PERIOD_MS = 200UL;
static const unsigned long SOFT_GUARD_REJECT_STEP = 2UL;
static const float SOFT_GUARD_OVERSPEED_MARGIN_MPS = 0.06f;
static const int SOFT_GUARD_PWM_RISE_PER_CYCLE = 25;

// Protection ultime cote Arduino contre une CMD_VEL corrompue.
// Le pont ROS actuel limite deja les roues a 0.5 m/s : cette limite ne change
// donc pas le fonctionnement normal.
static const float MAX_COMMAND_WHEEL_MPS = 0.50f;

// Rapport diagnostic des impulsions rejetees par le filtre ISR.
static const unsigned long ENCODER_DIAG_PERIOD_MS = 1000UL;

// Gains PI
float kp_left  = 300.0f;
float ki_left  = 6000.0f;
float kp_right = 300.0f;
float ki_right = 6000.0f;

static const float ITERM_LIMIT = 0.30f;

// PWM min pour vaincre le frottement
static const int PWM_DEADBAND = 0;

// ========= CONFIG IMU BNO055 =========
// Conversion exacte des degres vers les radians.
static const float DEG_TO_RAD_F = PI / 180.0f;

// Adresse frequente : 0x28. Si ton module est configure en 0x29, remplace par 0x29.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
bool imu_ok = false;

// ========= ETAT =========
//
// Compteurs RAW : uniquement ecrits dans les ISR.
// Compteurs VALIDES : seuls ceux-ci sont publies dans ENC,left,right.
// Une rafale impossible n'entre donc plus dans l'odometrie ROS.
volatile long left_raw_ticks  = 0;
volatile long right_raw_ticks = 0;

volatile unsigned long last_left_edge_us  = 0;
volatile unsigned long last_right_edge_us = 0;
volatile unsigned long rejected_left_edges  = 0;
volatile unsigned long rejected_right_edges = 0;

long prev_left_raw_ticks  = 0;
long prev_right_raw_ticks = 0;

long left_ticks  = 0;
long right_ticks = 0;

float target_left_mps  = 0.0f;
float target_right_mps = 0.0f;

float i_left  = 0.0f;
float i_right = 0.0f;

int left_dir_sign  = 0;
int right_dir_sign = 0;

unsigned long last_control_us = 0;
unsigned long last_odom_ms    = 0;
unsigned long last_imu_ms     = 0;
unsigned long last_cal_ms     = 0;
unsigned long last_cmd_ms     = 0;

// V23 soft guard.
unsigned long prev_control_rejected_left = 0;
unsigned long prev_control_rejected_right = 0;
unsigned long last_soft_guard_diag_ms = 0;
int last_pwm_left = 0;
int last_pwm_right = 0;
unsigned long last_encoder_diag_ms = 0;
unsigned long last_reported_rejected_left = 0;
unsigned long last_reported_rejected_right = 0;
long last_reported_raw_left = 0;
long last_reported_raw_right = 0;

String serial_line;

// Bits de diagnostic V23.
static const uint8_t SG_NONE       = 0;
static const uint8_t SG_INVALID    = 1 << 0;  // delta encodeur impossible
static const uint8_t SG_REJECTED   = 1 << 1;  // fronts rejetes pendant le cycle
static const uint8_t SG_OVERSPEED  = 1 << 2;  // vitesse mesuree trop haute
static const uint8_t SG_DIRECTION  = 1 << 3;  // PI voulait inverser sans consigne
static const uint8_t SG_SLEW       = 1 << 4;  // montee PWM limitee
static const uint8_t SG_SIGNCHANGE = 1 << 5;  // passage par zero lors inversion demandee

// ========= UTILS =========
float clampf(float x, float mn, float mx) {
  if (x < mn) return mn;
  if (x > mx) return mx;
  return x;
}

unsigned long calculateEncoderMinEdgeUs(float target_mps) {
  const float distance_per_tick_m =
      (PI * WHEEL_DIAMETER_M) / (float)TICKS_PER_WHEEL_REV;

  float v_allowed =
      ENCODER_FILTER_GAIN * fabs(target_mps) + ENCODER_FILTER_MARGIN_MPS;

  v_allowed = clampf(
      v_allowed,
      ENCODER_FILTER_MIN_ALLOWED_MPS,
      ENCODER_FILTER_MAX_ALLOWED_MPS
  );

  float edge_us = (distance_per_tick_m / v_allowed) * 1000000.0f;

  // Garde-fous absolus coherent avec les bornes 0.20 / 0.65 m/s.
  edge_us = clampf(edge_us, 400.0f, 1354.0f);

  return (unsigned long)(edge_us + 0.5f);
}

void updateDynamicEncoderThresholds(float left_target_mps,
                                    float right_target_mps) {
  const unsigned long new_left =
      calculateEncoderMinEdgeUs(left_target_mps);
  const unsigned long new_right =
      calculateEncoderMinEdgeUs(right_target_mps);

  // Variables 32 bits partagees avec les ISR : ecriture atomique.
  noInterrupts();
  encoder_min_edge_left_us = new_left;
  encoder_min_edge_right_us = new_right;
  interrupts();
}

void snapshotRawEncoderCounts(long &l_raw, long &r_raw) {
  noInterrupts();
  l_raw = left_raw_ticks;
  r_raw = right_raw_ticks;
  interrupts();
}

void syncRawEncoderBaseline() {
  long l_raw = 0;
  long r_raw = 0;
  snapshotRawEncoderCounts(l_raw, r_raw);
  prev_left_raw_ticks = l_raw;
  prev_right_raw_ticks = r_raw;
}

void stopMotors() {
  microM.Motors(0, 0, 0, 0);
  left_dir_sign = 0;
  right_dir_sign = 0;
  last_pwm_left = 0;
  last_pwm_right = 0;

  // IMPORTANT : ne jamais remettre la reference encodeur a zero alors que les
  // compteurs d'interruption restent cumulatifs. On se resynchronise sur le RAW
  // actuel afin d'eviter un faux delta gigantesque au redemarrage suivant.
  syncRawEncoderBaseline();

  i_left  = 0.0f;
  i_right = 0.0f;
}

long maxPlausibleTicksForStep(float target_mps, float dt) {
  float limit_mps = fabs(target_mps) + ENCODER_TARGET_MARGIN_MPS;
  if (fabs(target_mps) < 0.001f) {
    limit_mps = ENCODER_ZERO_TARGET_ALLOW_MPS;
  }
  limit_mps = clampf(limit_mps, ENCODER_ZERO_TARGET_ALLOW_MPS, ENCODER_ABSOLUTE_MAX_MPS);

  const float wheel_circumference_m = 2.0f * PI * WHEEL_RADIUS_M;
  float ticks_f = (limit_mps * dt / wheel_circumference_m) * (float)TICKS_PER_WHEEL_REV;
  long ticks = (long)ceil(ticks_f) + ENCODER_DELTA_MARGIN_TICKS;
  if (ticks < ENCODER_DELTA_MARGIN_TICKS) ticks = ENCODER_DELTA_MARGIN_TICKS;
  return ticks;
}

void reportEncoderGlitchSoft(const char* side, long dl_raw, long dr_raw,
                            long max_left, long max_right) {
  // V23 : diagnostic uniquement.
  // On ne touche NI aux consignes, NI aux integrales, NI aux moteurs ici.
  Serial.print(F("WARN,ENCODER_GLITCH_SOFT,"));
  Serial.print(side);
  Serial.print(F(",dl="));
  Serial.print(dl_raw);
  Serial.print(F(",dr="));
  Serial.print(dr_raw);
  Serial.print(F(",maxL="));
  Serial.print(max_left);
  Serial.print(F(",maxR="));
  Serial.println(max_right);
}


void reportRejectedEncoderEdges() {
  const unsigned long now_ms = millis();
  if (now_ms - last_encoder_diag_ms < ENCODER_DIAG_PERIOD_MS) return;
  last_encoder_diag_ms = now_ms;

  long l_raw, r_raw;
  unsigned long l_rej, r_rej;
  noInterrupts();
  l_raw = left_raw_ticks;
  r_raw = right_raw_ticks;
  l_rej = rejected_left_edges;
  r_rej = rejected_right_edges;
  interrupts();

  const long accepted_l_window = l_raw - last_reported_raw_left;
  const long accepted_r_window = r_raw - last_reported_raw_right;
  const unsigned long rejected_l_window = l_rej - last_reported_rejected_left;
  const unsigned long rejected_r_window = r_rej - last_reported_rejected_right;

  // Diagnostic utile pendant les essais : aucun changement du protocole de
  // commande ni du PI. Une ligne INFO/s maximum quand le robot est commande
  // ou quand au moins un front parasite a ete rejete.
  if (fabs(target_left_mps) >= 0.001f || fabs(target_right_mps) >= 0.001f ||
      rejected_l_window > 0 || rejected_r_window > 0) {
    Serial.print("INFO,ENCODER_DIAG,accL=");
    Serial.print(accepted_l_window);
    Serial.print(",accR=");
    Serial.print(accepted_r_window);
    Serial.print(",rejL=");
    Serial.print(rejected_l_window);
    Serial.print(",rejR=");
    Serial.print(rejected_r_window);
    Serial.print(",totalRejL=");
    Serial.print(l_rej);
    Serial.print(",totalRejR=");
    Serial.print(r_rej);
    Serial.print(",targetL=");
    Serial.print(target_left_mps, 3);
    Serial.print(",targetR=");
    Serial.print(target_right_mps, 3);
    Serial.print(",minEdgeL_us=");
    Serial.print(encoder_min_edge_left_us);
    Serial.print(",minEdgeR_us=");
    Serial.println(encoder_min_edge_right_us);
  }

  // On conserve aussi le WARN historique si le cumul de rejets a evolue.
  if (l_rej != last_reported_rejected_left || r_rej != last_reported_rejected_right) {
    Serial.print("WARN,ENCODER_EDGE_FILTER,left_rejected=");
    Serial.print(l_rej);
    Serial.print(",right_rejected=");
    Serial.println(r_rej);
  }

  last_reported_raw_left = l_raw;
  last_reported_raw_right = r_raw;
  last_reported_rejected_left = l_rej;
  last_reported_rejected_right = r_rej;
}

void setMotorsRaw(int pwm_left, int pwm_right) {
  pwm_left  = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  if (pwm_left > 0) {
    left_dir_sign = +1;
  } else if (pwm_left < 0) {
    left_dir_sign = -1;
  } else {
    left_dir_sign = 0;
  }

  if (pwm_right > 0) {
    right_dir_sign = +1;
  } else if (pwm_right < 0) {
    right_dir_sign = -1;
  } else {
    right_dir_sign = 0;
  }

  // Librairie microM : Motors(leftSpeed, rightSpeed, leftBrake, rightBrake)
  microM.Motors(pwm_left, pwm_right, 0, 0);

  // V23 : reference du dernier effort réellement applique.
  last_pwm_left = pwm_left;
  last_pwm_right = pwm_right;
}

int applyDeadband(int pwm) {
  if (pwm == 0) return 0;
  if (pwm > 0) return max(pwm, PWM_DEADBAND);
  return min(pwm, -PWM_DEADBAND);
}


int signOfFloat(float x) {
  if (x > 0.001f) return +1;
  if (x < -0.001f) return -1;
  return 0;
}

int signOfInt(int x) {
  if (x > 0) return +1;
  if (x < 0) return -1;
  return 0;
}

int applySoftPwmGuard(int candidate_pwm,
                      int previous_pwm,
                      float target_mps,
                      bool freeze_increase,
                      uint8_t &guard_mask) {
  const int target_sign = signOfFloat(target_mps);

  if (target_sign == 0) {
    return 0;
  }

  // Le PI n'est jamais autorise a creer de lui-meme une inversion non demandee.
  // Si Nav2 demande réellement une inversion, target_sign change et elle reste autorisee.
  if (candidate_pwm != 0 && signOfInt(candidate_pwm) != target_sign) {
    guard_mask |= SG_DIRECTION;
    candidate_pwm = 0;
  }

  const int prev_sign = signOfInt(previous_pwm);
  const int cand_sign = signOfInt(candidate_pwm);

  // Si la consigne a change de sens et qu'un ancien PWM oppose subsiste :
  // un seul cycle de 10 ms par zero, puis la nouvelle direction est autorisee.
  if (previous_pwm != 0 && prev_sign != target_sign) {
    guard_mask |= SG_SIGNCHANGE;
    return 0;
  }

  // Si l'encodeur est suspect / la roue est deja trop rapide :
  // aucune augmentation d'effort, mais toute diminution est autorisee.
  if (freeze_increase &&
      previous_pwm != 0 &&
      cand_sign == prev_sign &&
      abs(candidate_pwm) > abs(previous_pwm)) {
    candidate_pwm = previous_pwm;
  }

  // Limiteur general de montee : +25 PWM / 10 ms max.
  // Une baisse est immediate.
  if (candidate_pwm != 0 &&
      previous_pwm != 0 &&
      cand_sign == prev_sign &&
      abs(candidate_pwm) > abs(previous_pwm) + SOFT_GUARD_PWM_RISE_PER_CYCLE) {
    const int limited_mag =
        abs(previous_pwm) + SOFT_GUARD_PWM_RISE_PER_CYCLE;
    candidate_pwm = target_sign * limited_mag;
    guard_mask |= SG_SLEW;
  }

  // Depuis zero, on limite aussi le premier saut.
  if (previous_pwm == 0 &&
      abs(candidate_pwm) > SOFT_GUARD_PWM_RISE_PER_CYCLE) {
    candidate_pwm = target_sign * SOFT_GUARD_PWM_RISE_PER_CYCLE;
    guard_mask |= SG_SLEW;
  }

  return constrain(candidate_pwm, -255, 255);
}

void reportSoftGuard(float measL, float measR,
                     int pwmPiL, int pwmPiR,
                     int pwmOutL, int pwmOutR,
                     unsigned long rejStepL,
                     unsigned long rejStepR,
                     uint8_t guardL,
                     uint8_t guardR) {
  if (guardL == SG_NONE && guardR == SG_NONE) return;

  const unsigned long now_ms = millis();
  if (now_ms - last_soft_guard_diag_ms < SOFT_GUARD_DIAG_PERIOD_MS) return;
  last_soft_guard_diag_ms = now_ms;

  Serial.print(F("INFO,SOFT_GUARD,gL="));
  Serial.print(guardL);
  Serial.print(F(",gR="));
  Serial.print(guardR);
  Serial.print(F(",targetL="));
  Serial.print(target_left_mps, 4);
  Serial.print(F(",targetR="));
  Serial.print(target_right_mps, 4);
  Serial.print(F(",measL="));
  Serial.print(measL, 4);
  Serial.print(F(",measR="));
  Serial.print(measR, 4);
  Serial.print(F(",pwmPI_L="));
  Serial.print(pwmPiL);
  Serial.print(F(",pwmPI_R="));
  Serial.print(pwmPiR);
  Serial.print(F(",pwmOUT_L="));
  Serial.print(pwmOutL);
  Serial.print(F(",pwmOUT_R="));
  Serial.print(pwmOutR);
  Serial.print(F(",rejStepL="));
  Serial.print(rejStepL);
  Serial.print(F(",rejStepR="));
  Serial.println(rejStepR);
}

// ========= ENCODEURS =========
void isrLeft() {
  const unsigned long now_us = micros();
  const unsigned long dt_us = now_us - last_left_edge_us;

  if (last_left_edge_us == 0 || dt_us >= encoder_min_edge_left_us) {
    left_raw_ticks++;
    last_left_edge_us = now_us;
  } else {
    rejected_left_edges++;
  }
}

void isrRight() {
  const unsigned long now_us = micros();
  const unsigned long dt_us = now_us - last_right_edge_us;

  if (last_right_edge_us == 0 || dt_us >= encoder_min_edge_right_us) {
    right_raw_ticks++;
    last_right_edge_us = now_us;
  } else {
    rejected_right_edges++;
  }
}

// ========= SERIE =========
void rejectCmdVel(const char* reason,
                  float requested_left,
                  float requested_right) {
  Serial.print(F("WARN,CMD_VEL_REJECTED,reason="));
  Serial.print(reason);
  Serial.print(F(",requestedL="));
  Serial.print(requested_left, 3);
  Serial.print(F(",requestedR="));
  Serial.println(requested_right, 3);

  // Une trame hors plage ne doit JAMAIS etre transformee en une consigne
  // moteur valide. On arrete immediatement et on efface tout windup PI.
  target_left_mps  = 0.0f;
  target_right_mps = 0.0f;
  i_left  = 0.0f;
  i_right = 0.0f;

  updateDynamicEncoderThresholds(0.0f, 0.0f);
  stopMotors();

  // On considere la trame comme traitee. La prochaine CMD_VEL correcte
  // pourra reprendre normalement au cycle suivant.
  last_cmd_ms = millis();
}

void handleLine(const String& line) {
  if (!line.startsWith("CMD_VEL,")) {
    return;
  }

  const int p1 = line.indexOf(',');
  const int p2 = line.indexOf(',', p1 + 1);
  const int p3 = line.indexOf(',', p2 + 1);

  // Format strict : CMD_VEL,left,right
  // Une trame mal formee ne doit pas laisser une ancienne consigne active.
  if (p1 < 0 || p2 < 0 || p3 >= 0) {
    rejectCmdVel("FORMAT", 0.0f, 0.0f);
    return;
  }

  const float requested_left  = line.substring(p1 + 1, p2).toFloat();
  const float requested_right = line.substring(p2 + 1).toFloat();

  // Le pont ROS limite deja chaque roue a +/-0.50 m/s.
  // IMPORTANT : on REJETTE la trame hors plage au lieu de la clamper.
  // Exemple observe :
  //   Raspberry : CMD_VEL,0.088,0.072
  //   Arduino   : 88.000,0.072
  // L'ancien clamp transformait 88.0 en +0.5 m/s et chargeait brutalement le PI.
  if (isnan(requested_left) ||
      isnan(requested_right) ||
      fabs(requested_left) > MAX_COMMAND_WHEEL_MPS + 0.001f ||
      fabs(requested_right) > MAX_COMMAND_WHEEL_MPS + 0.001f) {
    rejectCmdVel("OUT_OF_RANGE", requested_left, requested_right);
    return;
  }

  const float new_target_left  = requested_left;
  const float new_target_right = requested_right;

  // Si la CMD demande réellement une inversion de roue, on efface seulement
  // l'integrale de cette roue pour ne pas transporter le windup de l'ancien sens.
  if (signOfFloat(new_target_left) != 0 &&
      signOfFloat(target_left_mps) != 0 &&
      signOfFloat(new_target_left) != signOfFloat(target_left_mps)) {
    i_left = 0.0f;
  }

  if (signOfFloat(new_target_right) != 0 &&
      signOfFloat(target_right_mps) != 0 &&
      signOfFloat(new_target_right) != signOfFloat(target_right_mps)) {
    i_right = 0.0f;
  }

  target_left_mps  = new_target_left;
  target_right_mps = new_target_right;

  updateDynamicEncoderThresholds(target_left_mps, target_right_mps);
  last_cmd_ms = millis();

  if (fabs(target_left_mps) < 0.001f &&
      fabs(target_right_mps) < 0.001f) {
    stopMotors();
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_line.length() > 0) {
        handleLine(serial_line);
        serial_line = "";
      }
    } else {
      serial_line += c;
      if (serial_line.length() > 100) {
        serial_line = "";
      }
    }
  }
}

// ========= REGULATION =========
void controlStep() {
  unsigned long now_us = micros();
  if (now_us - last_control_us < CONTROL_PERIOD_US) return;

  float dt = (now_us - last_control_us) * 1e-6f;
  last_control_us = now_us;
  if (dt <= 0.0f) return;

  // Timeout de communication : arret moteur et remise a zero de l'integrale.
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    const bool was_moving =
        fabs(target_left_mps) >= 0.001f || fabs(target_right_mps) >= 0.001f;
    target_left_mps = 0.0f;
    target_right_mps = 0.0f;
    if (was_moving) {
      updateDynamicEncoderThresholds(0.0f, 0.0f);
    }
    i_left = 0.0f;
    i_right = 0.0f;
  }

  // Lecture atomique des compteurs RAW + rejets ISR.
  long l_raw = 0;
  long r_raw = 0;
  unsigned long rejL_total = 0;
  unsigned long rejR_total = 0;

  noInterrupts();
  l_raw = left_raw_ticks;
  r_raw = right_raw_ticks;
  rejL_total = rejected_left_edges;
  rejR_total = rejected_right_edges;
  interrupts();

  long dl_raw = l_raw - prev_left_raw_ticks;
  long dr_raw = r_raw - prev_right_raw_ticks;
  prev_left_raw_ticks = l_raw;
  prev_right_raw_ticks = r_raw;

  const unsigned long rejStepL =
      rejL_total - prev_control_rejected_left;
  const unsigned long rejStepR =
      rejR_total - prev_control_rejected_right;
  prev_control_rejected_left = rejL_total;
  prev_control_rejected_right = rejR_total;

  uint8_t guardL = SG_NONE;
  uint8_t guardR = SG_NONE;

  // Controle de plausibilite.
  // V23 : on jette uniquement le DELTA impossible, sans annuler la CMD
  // et sans couper les moteurs.
  const long max_dl = maxPlausibleTicksForStep(target_left_mps, dt);
  const long max_dr = maxPlausibleTicksForStep(target_right_mps, dt);

  const bool invalidL = labs(dl_raw) > max_dl;
  const bool invalidR = labs(dr_raw) > max_dr;

  if (invalidL || invalidR) {
    const char* side = "BOTH";
    if (invalidL && !invalidR) side = "LEFT";
    if (invalidR && !invalidL) side = "RIGHT";
    reportEncoderGlitchSoft(side, dl_raw, dr_raw, max_dl, max_dr);
  }

  if (invalidL) {
    dl_raw = 0;
    guardL |= SG_INVALID;
  }
  if (invalidR) {
    dr_raw = 0;
    guardR |= SG_INVALID;
  }

  // Seulement les deltas plausibles deviennent des ticks ROS.
  left_ticks += dl_raw;
  right_ticks += dr_raw;

  float left_rev  = (float)dl_raw / (float)TICKS_PER_WHEEL_REV;
  float right_rev = (float)dr_raw / (float)TICKS_PER_WHEEL_REV;

  float left_mps_meas  =
      left_rev * 2.0f * PI * WHEEL_RADIUS_M / dt;
  float right_mps_meas =
      right_rev * 2.0f * PI * WHEEL_RADIUS_M / dt;

  // Encodeurs 1 voie : signe deduit du dernier effort moteur applique.
  left_mps_meas  *= (float)left_dir_sign;
  right_mps_meas *= (float)right_dir_sign;

  // Arret franc si consignes nulles.
  if (fabs(target_left_mps) < 0.001f &&
      fabs(target_right_mps) < 0.001f) {
    i_left = 0.0f;
    i_right = 0.0f;
    stopMotors();
    return;
  }

  // ========= REGULATION PI HISTORIQUE VALIDEE =========
  //
  // Gains INCHANGES :
  //   kp = 300, ki = 6000, ITERM_LIMIT = 0.30
  //
  // V23 ajoute seulement un anti-windup conditionnel autour du PI.
  const float i_left_before = i_left;
  const float i_right_before = i_right;

  float eL = target_left_mps  - left_mps_meas;
  float eR = target_right_mps - right_mps_meas;

  i_left  += eL * dt;
  i_right += eR * dt;

  i_left  = clampf(i_left,  -ITERM_LIMIT, ITERM_LIMIT);
  i_right = clampf(i_right, -ITERM_LIMIT, ITERM_LIMIT);

  // Une rafale de fronts rejetes signifie que la mesure acceptee peut
  // SOUS-estimer la vraie vitesse. Il ne faut surtout pas charger davantage I.
  if (rejStepL >= SOFT_GUARD_REJECT_STEP) {
    guardL |= SG_REJECTED;
  }
  if (rejStepR >= SOFT_GUARD_REJECT_STEP) {
    guardR |= SG_REJECTED;
  }

  // Survitesse claire par rapport a la consigne.
  if (!invalidL &&
      fabs(left_mps_meas) >
          fabs(target_left_mps) + SOFT_GUARD_OVERSPEED_MARGIN_MPS) {
    guardL |= SG_OVERSPEED;
  }
  if (!invalidR &&
      fabs(right_mps_meas) >
          fabs(target_right_mps) + SOFT_GUARD_OVERSPEED_MARGIN_MPS) {
    guardR |= SG_OVERSPEED;
  }

  const bool freezeL =
      (guardL & (SG_INVALID | SG_REJECTED | SG_OVERSPEED)) != 0;
  const bool freezeR =
      (guardR & (SG_INVALID | SG_REJECTED | SG_OVERSPEED)) != 0;

  if (freezeL) i_left = i_left_before;
  if (freezeR) i_right = i_right_before;

  // Recalcul PI avec l'integrale éventuellement gelee.
  float uL = kp_left  * eL + ki_left  * i_left;
  float uR = kp_right * eR + ki_right * i_right;

  int pwmPiL = (int)clampf(uL, -255.0f, 255.0f);
  int pwmPiR = (int)clampf(uR, -255.0f, 255.0f);

  pwmPiL = applyDeadband(pwmPiL);
  pwmPiR = applyDeadband(pwmPiR);

  // V23 :
  // - pas d'inversion non demandee ;
  // - pas d'augmentation si encodeur suspect/survitesse ;
  // - montee PWM lissee ;
  // - baisse immediate.
  int pwmL = applySoftPwmGuard(
      pwmPiL, last_pwm_left, target_left_mps, freezeL, guardL);
  int pwmR = applySoftPwmGuard(
      pwmPiR, last_pwm_right, target_right_mps, freezeR, guardR);

  reportSoftGuard(
      left_mps_meas, right_mps_meas,
      pwmPiL, pwmPiR,
      pwmL, pwmR,
      rejStepL, rejStepR,
      guardL, guardR);

  setMotorsRaw(pwmL, pwmR);
}

// ========= TELEMETRIE ENCODEURS =========
void sendEncoderFrame() {
  unsigned long now_ms = millis();
  if (now_ms - last_odom_ms < ODOM_PERIOD_MS) return;
  last_odom_ms = now_ms;

  // Compteurs deja valides par controlStep(); protocole ENC inchange.
  long l = left_ticks;
  long r = right_ticks;

  Serial.print("ENC,");
  Serial.print(l);
  Serial.print(",");
  Serial.println(r);
}

// ========= TELEMETRIE IMU =========
void sendImuFrame() {
  if (!imu_ok) return;

  unsigned long now_ms = millis();
  if (now_ms - last_imu_ms < IMU_PERIOD_MS) return;
  last_imu_ms = now_ms;

  // Données brutes dans le repère physique du BNO055.
  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> gyro =
      bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linacc =
      bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> mag =
      bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  /*
   * Montage vertical déterminé expérimentalement.
   * Pendant une rotation à plat vers la gauche, l'axe X brut
   * du BNO055 devient fortement négatif.
   *
   * Correspondance des vecteurs :
   *   X_robot =  Y_capteur
   *   Y_robot = -Z_capteur
   *   Z_robot = -X_capteur
   *
   * Ainsi, une rotation vers la gauche donne Z_robot positif.
   */

  // getVector(VECTOR_GYROSCOPE) renvoie des degres/s avec la librairie Adafruit.
  // Conversion en rad/s AVANT publication ROS.
  const float gx_robot = gyro.y()  * DEG_TO_RAD_F;
  const float gy_robot = -gyro.z() * DEG_TO_RAD_F;
  const float gz_robot = -gyro.x() * DEG_TO_RAD_F;

  // VECTOR_LINEARACCEL : m/s^2, gravite retiree par la fusion BNO055.
  const float ax_robot = linacc.y();
  const float ay_robot = -linacc.z();
  const float az_robot = -linacc.x();

  // VECTOR_MAGNETOMETER : microteslas (uT).
  // Meme changement de repere que pour le gyro et l'acceleration.
  const float mx_robot = mag.y();
  const float my_robot = -mag.z();
  const float mz_robot = -mag.x();

  /*
   * Correction du quaternion correspondant au même changement de repère.
   *
   * q_monde_robot = q_monde_capteur * q_capteur_robot
   * q_capteur_robot = (0.5, -0.5, -0.5, 0.5)
   */
  const float c = 0.5f;

  float qw_robot = c * (q.w() + q.x() + q.y() - q.z());
  float qx_robot = c * (-q.w() + q.x() + q.y() + q.z());
  float qy_robot = c * (-q.w() - q.x() + q.y() - q.z());
  float qz_robot = c * (q.w() - q.x() + q.y() + q.z());

  // Normalisation de sécurité.
  const float qnorm = sqrt(
      qw_robot * qw_robot +
      qx_robot * qx_robot +
      qy_robot * qy_robot +
      qz_robot * qz_robot
  );

  if (qnorm > 0.000001f) {
    qw_robot /= qnorm;
    qx_robot /= qnorm;
    qy_robot /= qnorm;
    qz_robot /= qnorm;
  } else {
    qw_robot = 1.0f;
    qx_robot = 0.0f;
    qy_robot = 0.0f;
    qz_robot = 0.0f;
  }

  // Format envoyé au Raspberry, déjà exprimé dans le repère du robot :
  // IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
  Serial.print("IMU,");
  Serial.print(qw_robot, 6);
  Serial.print(",");
  Serial.print(qx_robot, 6);
  Serial.print(",");
  Serial.print(qy_robot, 6);
  Serial.print(",");
  Serial.print(qz_robot, 6);
  Serial.print(",");
  Serial.print(gx_robot, 6);
  Serial.print(",");
  Serial.print(gy_robot, 6);
  Serial.print(",");
  Serial.print(gz_robot, 6);
  Serial.print(",");
  Serial.print(ax_robot, 6);
  Serial.print(",");
  Serial.print(ay_robot, 6);
  Serial.print(",");
  Serial.println(az_robot, 6);

  // Trame magnetometre separee, en microteslas (uT), repere robot.
  // Le noeud Raspberry convertira uT -> T pour sensor_msgs/MagneticField.
  Serial.print("MAG,");
  Serial.print(mx_robot, 6);
  Serial.print(",");
  Serial.print(my_robot, 6);
  Serial.print(",");
  Serial.println(mz_robot, 6);
}

// ========= ETAT DE CALIBRATION BNO055 =========
void sendCalibrationFrame() {
  if (!imu_ok) return;

  const unsigned long now_ms = millis();
  if (now_ms - last_cal_ms < CAL_PERIOD_MS) return;
  last_cal_ms = now_ms;

  uint8_t system_cal = 0;
  uint8_t gyro_cal = 0;
  uint8_t accel_cal = 0;
  uint8_t mag_cal = 0;

  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  // CAL,systeme,gyroscope,accelerometre,magnetometre
  Serial.print("CAL,");
  Serial.print(system_cal);
  Serial.print(",");
  Serial.print(gyro_cal);
  Serial.print(",");
  Serial.print(accel_cal);
  Serial.print(",");
  Serial.println(mag_cal);
}

// ========= SETUP / LOOP =========
void setup() {
  Serial.begin(115200);

  // Encodeurs : memes interruptions physiques que le firmware historique,
  // mais broches explicites + polarisation anti-entree-flottante.
  pinMode(ENCODER_LEFT_PIN,  ENCODER_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(ENCODER_RIGHT_PIN, ENCODER_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN),  isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isrRight, RISING);

  // IMU BNO055
  Wire.begin();
  delay(100);

  if (bno.begin()) {
    imu_ok = true;
    bno.setExtCrystalUse(true);
    Serial.println("INFO,BNO055_OK");
    Serial.println("INFO,UNITS,CMD_VEL=m_s,IMU_GYRO=rad_s,IMU_LINACC=m_s2,MAG=uT,QUAT=dimensionless");
    Serial.println(F("INFO,V24_SOFT_GUARD_CMD_REJECT,pwmRise=25,rejectGuard=2,overspeedMargin=0.06,maxCmd=0.50"));
    Serial.println("INFO,AXES,Xrobot=Ysensor,Yrobot=-Zsensor,Zrobot=-Xsensor");
    Serial.println("INFO,ENCODER_SAFETY,pins=2_3,pullup=1,dynamic_edge_filter=1,max_cmd_mps=0.50,ticks_per_rev=348");
    Serial.println("INFO,ENCODER_FILTER_DYNAMIC,min_allowed=0.20,gain=1.50,margin=0.05,max_allowed=0.65");
    Serial.println("INFO,PI_PRESERVED,kp=300,ki=6000,iterm_limit=0.30");
  } else {
    imu_ok = false;
    Serial.println("WARN,BNO055_NOT_FOUND");
  }

  stopMotors();
  updateDynamicEncoderThresholds(0.0f, 0.0f);

  last_control_us = micros();
  last_odom_ms = millis();
  last_imu_ms = millis();
  last_cal_ms = millis();
  last_cmd_ms = millis();
}

void loop() {
  readSerialCommands();
  controlStep();
  sendEncoderFrame();
  sendImuFrame();
  sendCalibrationFrame();
  reportRejectedEncoderEdges();
}
