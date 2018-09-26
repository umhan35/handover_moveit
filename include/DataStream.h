#pragma once

#include <deque>
#include <ostream>
#include <iostream>
#include <sstream>

#include <numeric>
#include <vector>


// multiple windowed data
class DataStream {

public:
    explicit DataStream(unsigned long max_number_of_windows, unsigned long window_size) {
        this->max_number_of_windows = max_number_of_windows;
        this->window_size = window_size;

        this->max_size = max_number_of_windows + window_size;
    }

    void operator<<(const double new_data) {
        data.push_back(new_data);

        if (data.size() > this->max_size) {
            data.pop_front();
        }
    }

    double decreasing_percentage(float percentage, float min_num_windows) {
//        std::cout << to_string() << std::endl;

        bool has_at_least_specified_windows_of_data = data.size() > window_size + min_num_windows; // +1 for comparisons
        if ( ! has_at_least_specified_windows_of_data)
            return false;

        // calculate average values
        std::vector<double> average_values;
        get_window_averages(average_values);


        unsigned long count_decreasing = 0;
        for (uint64_t i = 1; i < average_values.size(); ++i) {
            auto curr = average_values.at(i), prev = average_values.at(i-1);
            if ( curr < prev  && prev - curr > 0.0001) {
//            if ( curr < prev) {
                count_decreasing++;
            }
        }

        unsigned long total_comparisons = num_of_windows() - 1; // -1 for comparisons
        bool result = count_decreasing / (float)total_comparisons > percentage;
//        printf("decreasing: %.2f (%lu/%lu)\n", count_decreasing / (float)total_comparisons, count_decreasing, total_comparisons);
        printf("%f  (decreasing percent)\n", count_decreasing / (float)total_comparisons);

        return result;
    }

    unsigned long num_of_windows() { return data.size() - window_size; }

    void get_window_averages(std::vector<double> &average_values) {
        unsigned long num_of_windows = this->num_of_windows();
        for (int curr_window = 0; curr_window < num_of_windows; ++curr_window) {
            auto begin_slot = data.begin();
            std::advance(begin_slot, curr_window);

            auto end_slot = data.begin();
            std::advance(end_slot, curr_window + window_size);

            auto sum = std::accumulate(begin_slot, end_slot, 0.0);
            auto avg = sum / window_size;
//            printf("average fn: %d: avg: %f\n", curr_window, avg);
            average_values.push_back(avg);
        }
    }

    double increasing_percentage_old(float percentage, float min_dif) {
        std::cout << to_string() << std::endl;
        if (data.size() <= 1)
            return false;

        unsigned long total = data.size();
        unsigned long count = 0;

        for (uint64_t i = 1; i < data.size(); ++i) {
            if (data.at(i) > data.at(i-1) && data.at(i) - data.at(i-1) > 0.01) {
                count++;
            }
        }

        bool result = count / (float)total > percentage;
        return result;
    }

private:
    unsigned long max_number_of_windows;
    unsigned long window_size;

    unsigned long max_size;

    std::deque<double> data;

    std::string to_string() {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < data.size(); ++i) {
            ss << data.at(i);
            if (i + 1 != data.size())
                ss << ", ";
        }
        ss << "]";
        return ss.str();
    }

};
