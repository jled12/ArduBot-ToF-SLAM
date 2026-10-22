#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "ArducamTOFCamera.hpp"

using namespace std::chrono_literals;
using namespace Arducam;

ArducamTOFCamera tof;

class TOFPublisher : public rclcpp::Node
{
public:
    TOFPublisher() : Node("arducam"), pointsize_(43200)
    {
        pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud>();
        pc2_msg_->points.resize(pointsize_);
        pc2_msg_->channels.resize(1);
        pc2_msg_->channels[0].name = "intensities";
        pc2_msg_->channels[0].values.resize(pointsize_);

        depth_msg_ = std::make_shared<std_msgs::msg::Float32MultiArray>();
        depth_msg_->layout.dim.resize(2);
        depth_msg_->layout.dim[0].label = "height";
        depth_msg_->layout.dim[0].size = 180;
        depth_msg_->layout.dim[0].stride = 43200;
        depth_msg_->layout.dim[1].label = "width";
        depth_msg_->layout.dim[1].size = 240;
        depth_msg_->layout.dim[1].stride = 240;

        scan_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg_->header.frame_id = scan_frame_id_;
        scan_msg_->angle_min = angle_min_;
        scan_msg_->angle_max = angle_max_;
        scan_msg_->angle_increment = angle_increment_;
        scan_msg_->time_increment = 0.0f;
        scan_msg_->scan_time = 0.0f;
        scan_msg_->range_min = range_min_;
        scan_msg_->range_max = range_max_;

        const auto beam_count = static_cast<size_t>(
            std::floor((angle_max_ - angle_min_) / angle_increment_) + 1.0f);
        scan_msg_->ranges.resize(beam_count, no_return_value_);
        scan_msg_->intensities.resize(beam_count, 0.0f);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
            "point_cloud",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        publisher_pc2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "point_cloud2", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        publisher_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        publisher_depth_ =
            this->create_publisher<std_msgs::msg::Float32MultiArray>("depth_frame", 10);

        initPc2Parameters();

        timer_ = this->create_wall_timer(
            100ms, std::bind(&TOFPublisher::update, this));
    }

private:
    void generateSensorData()
    {
        ArducamFrameBuffer *frame;
        do
        {
            frame = tof.requestFrame(200);
        } while (frame == nullptr);

        depth_frame.clear();
        depth_frame.reserve(pointsize_);

        std::fill(
            scan_msg_->ranges.begin(),
            scan_msg_->ranges.end(),
            no_return_value_);

        std::fill(
            scan_msg_->intensities.begin(),
            scan_msg_->intensities.end(),
            0.0f);

        float *depth_ptr = (float *)frame->getData(FrameType::DEPTH_FRAME);
        float *amplitude_ptr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);

        unsigned long int pos = 0;

        for (int row_idx = 0; row_idx < 180; row_idx++)
        {
            for (int col_idx = 0; col_idx < 240; col_idx++, pos++)
            {
                if (amplitude_ptr[pos] > 80)
                {
                    // Arducam DEPTH_FRAME is interpreted here as the radial
                    // camera-to-point distance, not directly as axial Z.
                    // Reconstruct the point on the normalized camera ray using
                    // the current B0410 field of view (62.8 deg x 37.9 deg).
                    const float range = depth_ptr[pos];
                    const float nx = (cx_ - static_cast<float>(col_idx)) / fx_;
                    const float ny = (cy_ - static_cast<float>(row_idx)) / fy_;
                    const float ray_norm = std::sqrt(1.0f + nx * nx + ny * ny);

                    const float z = range / ray_norm;
                    const float x = nx * z;
                    const float y = ny * z;

                    pc2_msg_->points[pos].x = x;
                    pc2_msg_->points[pos].y = y;
                    pc2_msg_->points[pos].z = z;
                    pc2_msg_->channels[0].values[pos] = z;

                    // Keep /depth_frame as the raw sensor range for diagnostics.
                    depth_frame.push_back(range);

                    // Equivalent to the previous Python PointCloud -> LaserScan conversion:
                    // forward = z, lateral = x.
                    if (z <= 0.0f)
                        continue;

                    if (y < y_min_ || y > y_max_)
                        continue;

                    if (std::fabs(x) > lateral_ratio_max_ * z)
                        continue;

                    const float distance = std::hypot(z, x);
                    if (distance < range_min_ || distance > range_max_)
                        continue;

                    const float angle = std::atan2(x, z);
                    if (angle < angle_min_ || angle > angle_max_)
                        continue;

                    const int index = static_cast<int>(
                        std::lround((angle - angle_min_) / angle_increment_));

                    if (index < 0 ||
                        index >= static_cast<int>(scan_msg_->ranges.size()))
                        continue;

                    const size_t idx = static_cast<size_t>(index);
                    float &current = scan_msg_->ranges[idx];
                    if (current > range_max_ || distance < current)
                    {
                        current = distance;
                        scan_msg_->intensities[idx] = amplitude_ptr[pos];
                    }
                }
                else
                {
                    pc2_msg_->points[pos].x = 0.0f;
                    pc2_msg_->points[pos].y = 0.0f;
                    pc2_msg_->points[pos].z = 0.0f;
                    pc2_msg_->channels[0].values[pos] = 0.0f;
                    depth_frame.push_back(0.0f);
                }
            }
        }

        applySpeckleFilter();

        tof.releaseFrame(frame);

        pc2_msg_->header.frame_id = frame_id_;
        depth_msg_->data = depth_frame;
    }

    void applySpeckleFilter()
    {
        const std::vector<float> original = scan_msg_->ranges;

        for (size_t i = 0; i < original.size(); ++i)
        {
            const float r = original[i];
            if (r > range_max_)
                continue;

            int neighbor_count = 0;

            const int begin = std::max<int>(0, static_cast<int>(i) - speckle_window_);
            const int end = std::min<int>(
                static_cast<int>(original.size()) - 1,
                static_cast<int>(i) + speckle_window_);

            for (int j = begin; j <= end; ++j)
            {
                if (j == static_cast<int>(i))
                    continue;

                const float nr = original[static_cast<size_t>(j)];
                if (nr <= range_max_ &&
                    std::fabs(nr - r) <= speckle_max_range_difference_)
                {
                    ++neighbor_count;
                }
            }

            if (neighbor_count < speckle_min_neighbors_)
            {
                scan_msg_->ranges[i] = no_return_value_;
                scan_msg_->intensities[i] = 0.0f;
            }
        }
    }

    void update()
    {
        generateSensorData();

        const auto stamp = now();
        pc2_msg_->header.stamp = stamp;
        scan_msg_->header.stamp = stamp;

        // Publish the compact LaserScan first so SLAM receives the freshest data
        // without waiting for the large PointCloud message.
        publisher_scan_->publish(*scan_msg_);

        // Keep PointCloud for RViz/debug. It is no longer needed to build /scan.
        publisher_->publish(*pc2_msg_);
        publisher_depth_->publish(*depth_msg_);

        // 5 Hz 3D from 10 Hz acquisition; /scan remains at 10 Hz.
        if ((frame_counter_++ % 2) == 0) {
            publishCompactPointCloud2(stamp);
        }
    }

    // /point_cloud2 ONLY: inspect all 240x180 camera pixels at 5 Hz.
    // Full-resolution /scan, legacy /point_cloud and /depth_frame are untouched.
    // Filter ground + range in original sensor_frame before any thinning.
    // Apply the +1 degree pitch correction to the final PointCloud2 on the Pi.
    // Spatial thinning preferentially removes redundant points near the camera;
    // its leaves remain smaller than the 0.025 m OctoMap resolution.
    void initPc2Parameters()
    {
        pc2_min_y_ = declare_parameter<double>("pc2_min_y", -0.010);
        pc2_min_range_ = declare_parameter<double>("pc2_min_range", 0.17);
        pc2_max_range_ = declare_parameter<double>("pc2_max_range", 2.50);
        pc2_near_limit_ = declare_parameter<double>("pc2_near_limit", 0.70);
        pc2_far_limit_ = declare_parameter<double>("pc2_far_limit", 1.50);
        pc2_near_voxel_ = declare_parameter<double>("pc2_near_voxel", 0.015);
        pc2_mid_voxel_ = declare_parameter<double>("pc2_mid_voxel", 0.0125);
        pc2_far_voxel_ = declare_parameter<double>("pc2_far_voxel", 0.010);
        // Conservative Wi-Fi payload cap. 0 disables the cap; voxel filter
        // remains active either way. 10,000 xyz points x 12B x 5Hz ~= 4.8 Mbps
        // payload (actual Wi-Fi/DDS overhead is higher).
        pc2_max_points_ = declare_parameter<int>("pc2_max_points", 10000);
        pc2_pitch_correction_deg_ =
            declare_parameter<double>("pc2_pitch_correction_deg", 1.0);

        // Feature-preserving thinning for /point_cloud2 only.
        //
        // A valid pixel is DETAIL only when it is significantly CLOSER than
        // the robust median of its 5x5 neighbourhood. This preserves thin
        // foreground objects (chair legs, table legs, cables, edges...) while
        // avoiding the old behaviour where sensor noise made almost the whole
        // image look like a depth edge.
        pc2_foreground_abs_threshold_ =
            declare_parameter<double>("pc2_foreground_abs_threshold", 0.030);
        pc2_foreground_rel_threshold_ =
            declare_parameter<double>("pc2_foreground_rel_threshold", 0.012);
        pc2_median_radius_px_ =
            declare_parameter<int>("pc2_median_radius_px", 2);  // 5x5
        pc2_median_min_neighbors_ =
            declare_parameter<int>("pc2_median_min_neighbors", 8);
        pc2_foreground_min_far_neighbors_ =
            declare_parameter<int>("pc2_foreground_min_far_neighbors", 6);
        pc2_detail_halo_px_ =
            declare_parameter<int>("pc2_detail_halo_px", 1);

        pc2_surface_voxel_ =
            declare_parameter<double>("pc2_surface_voxel", 0.025);
        pc2_surface_temporal_phases_ =
            declare_parameter<int>("pc2_surface_temporal_phases", 4);

        if (!std::isfinite(pc2_min_y_) ||
            !std::isfinite(pc2_min_range_) ||
            !std::isfinite(pc2_max_range_) ||
            !std::isfinite(pc2_near_limit_) ||
            !std::isfinite(pc2_far_limit_) ||
            !std::isfinite(pc2_near_voxel_) ||
            !std::isfinite(pc2_mid_voxel_) ||
            !std::isfinite(pc2_far_voxel_) ||
            !std::isfinite(pc2_foreground_abs_threshold_) ||
            !std::isfinite(pc2_foreground_rel_threshold_) ||
            !std::isfinite(pc2_surface_voxel_) ||
            pc2_min_range_ < 0.0 || pc2_min_range_ >= pc2_max_range_ ||
            pc2_near_limit_ <= pc2_min_range_ ||
            pc2_near_limit_ >= pc2_far_limit_ ||
            pc2_far_limit_ >= pc2_max_range_ ||
            pc2_near_voxel_ < 0.001 || pc2_mid_voxel_ < 0.001 ||
            pc2_far_voxel_ < 0.001 ||
            pc2_near_voxel_ > 0.025 || pc2_mid_voxel_ > 0.025 ||
            pc2_far_voxel_ > 0.025 ||
            pc2_near_voxel_ < pc2_mid_voxel_ ||
            pc2_mid_voxel_ < pc2_far_voxel_ ||
            pc2_max_points_ < 0 || pc2_max_points_ > 43200 ||
            pc2_foreground_abs_threshold_ < 0.010 ||
            pc2_foreground_abs_threshold_ > 0.20 ||
            pc2_foreground_rel_threshold_ < 0.0 ||
            pc2_foreground_rel_threshold_ > 0.10 ||
            pc2_median_radius_px_ < 1 || pc2_median_radius_px_ > 2 ||
            pc2_median_min_neighbors_ < 4 || pc2_median_min_neighbors_ > 24 ||
            pc2_foreground_min_far_neighbors_ < 1 ||
            pc2_foreground_min_far_neighbors_ > 24 ||
            pc2_detail_halo_px_ < 0 || pc2_detail_halo_px_ > 2 ||
            pc2_surface_voxel_ < 0.010 || pc2_surface_voxel_ > 0.050 ||
            (pc2_surface_temporal_phases_ != 1 &&
             pc2_surface_temporal_phases_ != 2 &&
             pc2_surface_temporal_phases_ != 4)) {
            throw std::invalid_argument("Invalid adaptive PointCloud2 parameters");
        }

        if (!std::isfinite(pc2_pitch_correction_deg_) ||
            std::abs(pc2_pitch_correction_deg_) > 5.0) {
            throw std::invalid_argument(
                "pc2_pitch_correction_deg must be between -5 and +5 degrees");
        }

        constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
        const double pitch_rad = pc2_pitch_correction_deg_ * DEG_TO_RAD;
        pc2_cos_pitch_ = static_cast<float>(std::cos(pitch_rad));
        pc2_sin_pitch_ = static_cast<float>(std::sin(pitch_rad));

        RCLCPP_INFO(get_logger(),
            "PC2 median-foreground: 43200 pixels inspected, sensor_frame y>%.3fm, "
            "range [%.2f,%.2f]m; DETAIL voxels %.4f/%.4f/%.4fm; "
            "foreground if range < median(%dx%d) - max(%.3fm, %.1f%% median), "
            "with >=%d valid neighbours and >=%d farther neighbours; halo=%d px; "
            "SMOOTH voxel=%.3fm and 1/%d temporal phase; cap=%d points/frame, 5Hz. "
            "/scan unchanged; DETAIL has priority.",
            pc2_min_y_, pc2_min_range_, pc2_max_range_,
            pc2_near_voxel_, pc2_mid_voxel_, pc2_far_voxel_,
            2 * pc2_median_radius_px_ + 1,
            2 * pc2_median_radius_px_ + 1,
            pc2_foreground_abs_threshold_,
            100.0 * pc2_foreground_rel_threshold_,
            pc2_median_min_neighbors_,
            pc2_foreground_min_far_neighbors_,
            pc2_detail_halo_px_,
            pc2_surface_voxel_,
            pc2_surface_temporal_phases_,
            pc2_max_points_);
    }

    bool validPc2Point(size_t index) const
    {
        const auto &p = pc2_msg_->points[index];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.z) || p.z <= 0.0f || p.y <= pc2_min_y_) {
            return false;
        }
        const float range2 = p.x * p.x + p.y * p.y + p.z * p.z;
        return range2 >= pc2_min_range_ * pc2_min_range_ &&
               range2 <= pc2_max_range_ * pc2_max_range_;
    }

    // Unique 3D voxel key: 2 bits for near/mid/far, then signed 16-bit XYZ
    // cell indices. Coordinates are <= 2.5m and leaf size >= 0.01m,
    // so each signed index fits comfortably in 16 bits.
    static uint64_t voxelKey(float x, float y, float z,
                             unsigned band, double leaf)
    {
        const auto ix = static_cast<int16_t>(std::floor(x / leaf));
        const auto iy = static_cast<int16_t>(std::floor(y / leaf));
        const auto iz = static_cast<int16_t>(std::floor(z / leaf));
        return (static_cast<uint64_t>(band) << 48) |
               (static_cast<uint64_t>(static_cast<uint16_t>(ix)) << 32) |
               (static_cast<uint64_t>(static_cast<uint16_t>(iy)) << 16) |
               static_cast<uint64_t>(static_cast<uint16_t>(iz));
    }

    void publishCompactPointCloud2(const rclcpp::Time &stamp)
    {
        // Build a validity/range image once. This keeps the feature detector cheap
        // and does not touch /scan or the legacy point cloud.
        std::vector<uint8_t> valid(pointsize_, 0);
        std::vector<float> ranges(pointsize_, 0.0f);
        std::vector<uint8_t> foreground_seed(pointsize_, 0);
        std::vector<uint8_t> detail_mask(pointsize_, 0);

        size_t valid_count = 0;
        for (size_t i = 0; i < pointsize_; ++i) {
            if (!validPc2Point(i)) {
                continue;
            }
            const auto &p = pc2_msg_->points[i];
            const float range = std::sqrt(
                p.x * p.x + p.y * p.y + p.z * p.z);
            valid[i] = 1;
            ranges[i] = range;
            ++valid_count;
        }

        constexpr int width = 240;
        constexpr int height = 180;

        // Robust 5x5 foreground detector.
        //
        // We compare the centre pixel to the median of its valid neighbours
        // (centre excluded). A sloped/rough wall normally stays close to this
        // local median and is therefore SMOOTH. A thin object in front of the
        // wall remains significantly closer than the median and becomes DETAIL.
        //
        // The extra "far-neighbour" count rejects weak/local fluctuations:
        // enough surrounding samples must really lie behind the foreground
        // candidate before it is promoted to DETAIL.
        const int radius = pc2_median_radius_px_;
        size_t median_checked_count = 0;
        size_t insufficient_neighbours_count = 0;
        size_t foreground_seed_count = 0;

        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                const size_t idx =
                    static_cast<size_t>(row * width + col);

                if (!valid[idx]) {
                    continue;
                }

                std::array<float, 24> neighbour_ranges{};
                int neighbour_count = 0;

                const int r0 = std::max(0, row - radius);
                const int r1 = std::min(height - 1, row + radius);
                const int c0 = std::max(0, col - radius);
                const int c1 = std::min(width - 1, col + radius);

                for (int rr = r0; rr <= r1; ++rr) {
                    for (int cc = c0; cc <= c1; ++cc) {
                        if (rr == row && cc == col) {
                            continue;
                        }
                        const size_t j =
                            static_cast<size_t>(rr * width + cc);
                        if (!valid[j]) {
                            continue;
                        }
                        // radius is limited to 2, hence max 24 neighbours.
                        neighbour_ranges[
                            static_cast<size_t>(neighbour_count++)] = ranges[j];
                    }
                }

                if (neighbour_count < pc2_median_min_neighbors_) {
                    ++insufficient_neighbours_count;
                    continue;
                }

                ++median_checked_count;

                const int median_pos = neighbour_count / 2;
                std::nth_element(
                    neighbour_ranges.begin(),
                    neighbour_ranges.begin() + median_pos,
                    neighbour_ranges.begin() + neighbour_count);

                const float median_range =
                    neighbour_ranges[static_cast<size_t>(median_pos)];

                const float threshold = std::max(
                    static_cast<float>(pc2_foreground_abs_threshold_),
                    static_cast<float>(
                        pc2_foreground_rel_threshold_ * median_range));

                const float centre_range = ranges[idx];
                if (median_range - centre_range <= threshold) {
                    continue;
                }

                int far_neighbours = 0;
                const float far_limit = centre_range + threshold;
                for (int n = 0; n < neighbour_count; ++n) {
                    if (neighbour_ranges[static_cast<size_t>(n)] > far_limit) {
                        ++far_neighbours;
                    }
                }

                if (far_neighbours < pc2_foreground_min_far_neighbors_) {
                    continue;
                }

                foreground_seed[idx] = 1;
                ++foreground_seed_count;
            }
        }

        // One-pixel halo around true foreground seeds.
        // For a ~40 mm chair leg this keeps the thin object's complete local
        // footprint while no longer dilating noise over a 5x5 area as before.
        const int halo = pc2_detail_halo_px_;
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                const size_t idx =
                    static_cast<size_t>(row * width + col);

                if (!foreground_seed[idx]) {
                    continue;
                }

                const int r0 = std::max(0, row - halo);
                const int r1 = std::min(height - 1, row + halo);
                const int c0 = std::max(0, col - halo);
                const int c1 = std::min(width - 1, col + halo);

                for (int rr = r0; rr <= r1; ++rr) {
                    for (int cc = c0; cc <= c1; ++cc) {
                        const size_t j =
                            static_cast<size_t>(rr * width + cc);
                        if (valid[j]) {
                            detail_mask[j] = 1;
                        }
                    }
                }
            }
        }

        std::vector<size_t> detail_indices;
        std::vector<size_t> surface_indices;
        detail_indices.reserve(pointsize_ / 4);
        surface_indices.reserve(pointsize_ / 4);

        std::unordered_set<uint64_t> detail_cells;
        std::unordered_set<uint64_t> surface_cells;
        detail_cells.reserve(pointsize_ / 2);
        surface_cells.reserve(pointsize_ / 2);

        size_t detail_before_voxel = 0;
        size_t surface_valid = 0;
        size_t surface_phase_kept = 0;

        const unsigned phase_count =
            static_cast<unsigned>(pc2_surface_temporal_phases_);
        const unsigned current_phase =
            static_cast<unsigned>(pc2_publish_counter_ % phase_count);
        ++pc2_publish_counter_;

        // Pass 1: detail zones first. They keep the original fine adaptive
        // voxel sizes (10-15 mm) and are never sacrificed to smooth surfaces.
        for (size_t i = 0; i < pointsize_; ++i) {
            if (!valid[i] || !detail_mask[i]) {
                continue;
            }
            ++detail_before_voxel;
            const auto &p = pc2_msg_->points[i];
            const double range = ranges[i];
            const unsigned band = range < pc2_near_limit_ ? 0U :
                                  range < pc2_far_limit_ ? 1U : 2U;
            const double leaf = band == 0U ? pc2_near_voxel_ :
                                band == 1U ? pc2_mid_voxel_ :
                                             pc2_far_voxel_;
            const uint64_t key = voxelKey(p.x, p.y, p.z, band, leaf);
            if (detail_cells.insert(key).second) {
                detail_indices.push_back(i);
            }
        }

        // Pass 2: large/smooth surfaces. Keep only one spatial phase each
        // PointCloud2 frame, rotating the phase at 5 Hz, then use one voxel
        // equal to the OctoMap resolution. Thus walls/large blocks cost much
        // less per frame without reducing the final OctoMap resolution.
        for (size_t i = 0; i < pointsize_; ++i) {
            if (!valid[i] || detail_mask[i]) {
                continue;
            }
            ++surface_valid;

            const int row = static_cast<int>(i / width);
            const int col = static_cast<int>(i % width);

            unsigned pixel_phase = 0U;
            if (phase_count == 2U) {
                pixel_phase =
                    static_cast<unsigned>((row + col) & 1);
            } else if (phase_count == 4U) {
                pixel_phase =
                    (static_cast<unsigned>(row & 1) << 1) |
                    static_cast<unsigned>(col & 1);
            }

            if (pixel_phase != current_phase) {
                continue;
            }
            ++surface_phase_kept;

            const auto &p = pc2_msg_->points[i];
            const uint64_t key =
                voxelKey(p.x, p.y, p.z, 0U, pc2_surface_voxel_);
            if (surface_cells.insert(key).second) {
                surface_indices.push_back(i);
            }
        }

        const size_t detail_candidates = detail_indices.size();
        const size_t surface_candidates = surface_indices.size();
        const size_t candidates = detail_candidates + surface_candidates;
        const size_t budget = pc2_max_points_ == 0
            ? candidates
            : std::min(candidates, static_cast<size_t>(pc2_max_points_));

        // Detail always gets the budget first. Uniform selection is used only
        // if the DETAIL set itself exceeds the complete frame budget.
        const size_t detail_budget = std::min(detail_candidates, budget);
        const size_t surface_budget =
            std::min(surface_candidates, budget - detail_budget);

        std::vector<size_t> selected_indices;
        selected_indices.reserve(detail_budget + surface_budget);

        auto append_uniform = [&](const std::vector<size_t> &source,
                                  size_t take) {
            if (take == 0 || source.empty()) {
                return;
            }
            if (take >= source.size()) {
                selected_indices.insert(
                    selected_indices.end(), source.begin(), source.end());
                return;
            }
            for (size_t i = 0; i < take; ++i) {
                const size_t pos =
                    ((2 * i + 1) * source.size()) / (2 * take);
                selected_indices.push_back(source[pos]);
            }
        };

        append_uniform(detail_indices, detail_budget);
        append_uniform(surface_indices, surface_budget);

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = frame_id_; // unchanged sensor_frame TF
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.point_step = 12;
        cloud.fields.resize(3);
        const char *names[3] = {"x", "y", "z"};
        for (uint32_t i = 0; i < 3; ++i) {
            cloud.fields[i].name = names[i];
            cloud.fields[i].offset = i * 4;
            cloud.fields[i].datatype =
                sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[i].count = 1;
        }

        cloud.width =
            static_cast<uint32_t>(selected_indices.size());
        cloud.row_step = cloud.width * cloud.point_step;
        cloud.data.resize(
            selected_indices.size() * cloud.point_step);

        for (size_t i = 0; i < selected_indices.size(); ++i) {
            const auto &p =
                pc2_msg_->points[selected_indices[i]];
            const size_t offset = i * cloud.point_step;

            // Same +1 degree correction as before, after feature selection.
            // /scan remains strictly unchanged.
            const float x_out = p.x;
            const float y_out =
                p.y * pc2_cos_pitch_ + p.z * pc2_sin_pitch_;
            const float z_out =
                p.z * pc2_cos_pitch_ - p.y * pc2_sin_pitch_;

            std::memcpy(cloud.data.data() + offset, &x_out, 4);
            std::memcpy(cloud.data.data() + offset + 4, &y_out, 4);
            std::memcpy(cloud.data.data() + offset + 8, &z_out, 4);
        }

        publisher_pc2_->publish(cloud);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "PC2 median-FG: %zu valid; median checked=%zu, insufficient=%zu; "
            "FG seeds=%zu; DETAIL %zu raw -> %zu voxel -> %zu sent; "
            "SMOOTH %zu raw -> %zu phase(1/%u,p=%u) -> %zu voxel -> %zu sent; "
            "TOTAL %zu sent, %.1f kB/frame (~%.2f Mbit/s payload @5Hz); cap=%d",
            valid_count, median_checked_count, insufficient_neighbours_count,
            foreground_seed_count,
            detail_before_voxel, detail_candidates, detail_budget,
            surface_valid, surface_phase_kept, phase_count, current_phase,
            surface_candidates, surface_budget,
            selected_indices.size(),
            static_cast<double>(cloud.data.size()) / 1024.0,
            (selected_indices.size() * 12.0 * 5.0 * 8.0) / 1000000.0,
            pc2_max_points_);
    }

    double pc2_min_y_ = -0.010;
    double pc2_min_range_ = 0.17;
    double pc2_max_range_ = 2.50;
    double pc2_near_limit_ = 0.70;
    double pc2_far_limit_ = 1.50;
    double pc2_near_voxel_ = 0.015;
    double pc2_mid_voxel_ = 0.0125;
    double pc2_far_voxel_ = 0.010;
    int pc2_max_points_ = 10000;

    // Feature-preserving /point_cloud2 filtering:
    // robust local median + foreground detection.
    double pc2_foreground_abs_threshold_ = 0.030;
    double pc2_foreground_rel_threshold_ = 0.012;
    int pc2_median_radius_px_ = 2;          // 5x5 neighbourhood
    int pc2_median_min_neighbors_ = 8;
    int pc2_foreground_min_far_neighbors_ = 6;
    int pc2_detail_halo_px_ = 1;

    double pc2_surface_voxel_ = 0.025;
    int pc2_surface_temporal_phases_ = 4;
    uint64_t pc2_publish_counter_ = 0;

    // Correction de l'inclinaison mécanique de la ToF.
    // Appliquée uniquement au PointCloud2 compact final.
    double pc2_pitch_correction_deg_ = 1.0;
    float pc2_cos_pitch_ = 1.0f;
    float pc2_sin_pitch_ = 0.0f;

    std::string frame_id_ = "sensor_frame";
    std::string scan_frame_id_ = "laser_frame";

    std::vector<float> depth_frame;
    sensor_msgs::msg::PointCloud::SharedPtr pc2_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    std_msgs::msg::Float32MultiArray::SharedPtr depth_msg_;

    size_t pointsize_;
    uint64_t frame_counter_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_depth_;

    // Same scan settings as the existing Python converter.
    const float angle_min_ = -0.75f;
    const float angle_max_ = 0.75f;
    const float angle_increment_ = 0.01f;
    const float range_min_ = 0.17f;
    const float range_max_ = 3.5f;
    const float no_return_value_ = 3.5f;
    const float lateral_ratio_max_ = 0.8f;
    const float y_min_ = 0.01f;
    const float y_max_ = 0.025f;

    const float speckle_max_range_difference_ = 0.05f;
    const int speckle_min_neighbors_ = 3;
    const int speckle_window_ = 2;

    // B0410 geometry used by the validated FOV_SPEC_RADIAL preview.
    // Pixel centres run from 0..239 and 0..179, hence 119.5 / 89.5.
    static constexpr float cx_ = 119.5f;
    static constexpr float cy_ = 89.5f;
    static constexpr float horizontal_fov_deg_ = 62.8f;
    static constexpr float vertical_fov_deg_ = 37.9f;

    const float fx_ = 240.0f /
        (2.0f * std::tan(0.5f * static_cast<float>(M_PI) * horizontal_fov_deg_ / 180.0f));
    const float fy_ = 180.0f /
        (2.0f * std::tan(0.5f * static_cast<float>(M_PI) * vertical_fov_deg_ / 180.0f));
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (tof.open(Connection::CSI))
    {
        printf("initialize fail\n");
        exit(-1);
    }

    if (tof.start())
    {
        printf("start fail\n");
        exit(-1);
    }

    tof.setControl(CameraCtrl::RANGE, 4);

    printf("pointcloud + direct laserscan publisher start - radial XYZ projection 62.8x37.9 deg\n");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::spin(std::make_shared<TOFPublisher>());
    rclcpp::shutdown();

    tof.stop();
    tof.close();
    return 0;
}
