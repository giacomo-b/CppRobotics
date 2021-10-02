#include <doctest/doctest.h>
#include <robotics/common.h>

using namespace Robotics;

TEST_CASE("Linspace_integer")
{
    constexpr int a{0};
    constexpr int b{10};
    constexpr std::size_t N{11};
    std::vector<int> test_arr{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    std::vector<int> arr = linspace<int>(a, b, N);

    CHECK(arr == test_arr);
}

TEST_CASE("Linspace_negative_integer")
{
    constexpr int a{-10};
    constexpr int b{0};
    constexpr std::size_t N{11};
    std::vector<int> test_arr{-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0};

    std::vector<int> arr = linspace<int>(a, b, N);

    CHECK(arr == test_arr);
}

TEST_CASE("Linspace_decreasing_integer")
{
    constexpr int a{5};
    constexpr int b{0};
    constexpr std::size_t N{6};
    std::vector<int> test_arr{5, 4, 3, 2, 1, 0};

    std::vector<int> arr = linspace<int>(a, b, N);

    CHECK(arr == test_arr);
}

TEST_CASE("Linspace_double")
{
    constexpr double a{0.0};
    constexpr double b{1.2};
    constexpr double test_error_tolerance{0.001};
    std::vector<double> test_arr{0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2};
    std::vector<double> arr = linspace<double>(a, b, 7);
    CHECK(arr[0] == test_arr[0]);
    CHECK(arr[1] == doctest::Approx(test_arr[1]).epsilon(test_error_tolerance));
    CHECK(arr[2] == doctest::Approx(test_arr[2]).epsilon(test_error_tolerance));
    CHECK(arr[3] == doctest::Approx(test_arr[3]).epsilon(test_error_tolerance));
    CHECK(arr[4] == doctest::Approx(test_arr[4]).epsilon(test_error_tolerance));
    CHECK(arr[5] == doctest::Approx(test_arr[5]).epsilon(test_error_tolerance));
    CHECK(arr[6] == doctest::Approx(test_arr[6]).epsilon(test_error_tolerance));
}
