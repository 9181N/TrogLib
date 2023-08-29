#pragma once

namespace mkhlib {
    namespace utils {
        bool positive(int num) {
            return num >= 0;
        }

        bool negative(int num) {
            return num < 0;
        }
    }
}