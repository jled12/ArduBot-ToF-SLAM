import math
from dataclasses import dataclass


PROFILES = {
    'B2': {
        'heading_kcap': 0.60,
        'heading_kgyro': 0.16,
        'heading_bias_rad_s': -0.053,
    },
    'C1': {
        'heading_kcap': 0.60,
        'heading_kgyro': 0.16,
        'heading_bias_rad_s': -0.060,
    },
}


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class HeadingGains:
    kcap: float
    kgyro: float
    bias_rad_s: float


def profile_gains(profile: str) -> HeadingGains:
    key = profile.upper()
    if key not in PROFILES:
        raise ValueError(f'Profil inconnu {profile!r}. Profils autorisés : B2, C1.')
    values = PROFILES[key]
    return HeadingGains(
        kcap=float(values['heading_kcap']),
        kgyro=float(values['heading_kgyro']),
        bias_rad_s=float(values['heading_bias_rad_s']),
    )
