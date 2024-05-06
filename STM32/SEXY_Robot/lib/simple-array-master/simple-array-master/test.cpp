#include <iostream>
#include <UnitTest++.h>
/**
 * To use UnitTest++ in Eclipse:
 * https://stackoverflow.com/questions/9815131/eclipse-cdt-convert-a-normal-folder-to-a-source-folder-or-vice-versa
 * https://stackoverflow.com/questions/2424795/building-multiple-binaries-within-one-eclipse-project
 *
 * In Eclipse project's properties setting:
 *
 * [C/C++ Build] | [Settings]
 *
 *     GCC C++ Compiler
 *         Includes
 *             Include paths (-l) >> "${workspace_loc:/simple-array/unittest-cpp/UnitTest++}"
 *
 *     GCC C++ Linker
 *         Libraries
 *             Libraries (-l)           >> "UnitTest++"
 *             Library search path (-L) >> "${workspace_loc:/simple-array/unittest-cpp/UnitTest++/.libs}"
 *             Miscellaneous
 *                 Linker flags         >> -static
 *
 * @link: https://github.com/unittest-cpp/unittest-cpp/wiki/Macro-and-Parameter-Reference
 */

#include "SimpleArray.h"
typedef SimpleArray<uint8_t, int> array_t;

#define HEAP_SIZE  100
#define TEST_SIZE  20
#define TEST_COUNT 1000

/**
 * Test with Setup and Teardown
 */
class MyFixture {
  public:
    array_t arr1 = array_t(HEAP_SIZE);
    array_t arr2 = array_t(HEAP_SIZE);

    MyFixture() {
        for (int i = 0; i < TEST_SIZE; i++)
            arr1 += i;
        arr2 += arr1;

        //CHECK_ARRAY_EQUAL(arr1, arr2, TEST_SIZE);
        for (int i = 0; i < TEST_SIZE; i++)
            CHECK_EQUAL(arr1[i], arr2[i]);
    }
    ~MyFixture() {
    }
};

TEST_FIXTURE(MyFixture, ArrayEqual) {
    CHECK(arr1 == arr2);
}

TEST_FIXTURE(MyFixture, ArrayNotEqual) {
    array_t arr(HEAP_SIZE);
    arr = arr2;
    arr.remove(8, TEST_SIZE);
    CHECK(arr1 != arr);
}

TEST_FIXTURE(MyFixture, ItemAssignment) {
    array_t arr(HEAP_SIZE);
    uint8_t a[TEST_SIZE];
    for (int i = 0; i < TEST_SIZE; i++)
        arr += a[i] = i;
    for (int i = 0; i < TEST_SIZE; i++)
        CHECK(arr[i] == a[i]);
}

TEST_FIXTURE(MyFixture, Reliability) {
    array_t arr(HEAP_SIZE);
    arr = arr2;

    #if (HEAP_SIZE != 100) || (TEST_SIZE != 20) || (TEST_COUNT != 1000)
    #error "This test was prepared for the specific HEAP_SIZE, TEST_SIZE, and TEST_COUNT."
    #endif

    uint8_t ground_truth[] = {0, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 1, 2, 3, 4, 5, 6};

    for (int t = 0; t < TEST_COUNT; t++) {
        uint8_t tmp = arr[1];
        arr.remove(1, 10);

        for (int j = tmp; j < tmp + 10; j++) {
            arr += (j < TEST_SIZE) ? j : j - TEST_SIZE + 1;
        }
    }

    for (int i = 0; i < TEST_SIZE; i++) {
        CHECK(arr[i] == ground_truth[i]);
    }
}

TEST_FIXTURE(MyFixture, Insertion) {
    array_t arr(HEAP_SIZE);
    arr = arr2;

    #if (HEAP_SIZE != 100) || (TEST_SIZE != 20) || (TEST_COUNT != 1000)
    #error "This test was prepared for the specific HEAP_SIZE, TEST_SIZE, and TEST_COUNT."
    #endif

    uint8_t ground_truth_1[] = {
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
            99, 88, 77, 66, 55, 19};
    uint8_t ground_truth_2[] = {
            99, 88, 77, 66, 55,
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
            99, 88, 77, 66, 55, 19};
    uint8_t inserted[] = {99, 88, 77, 66, 55};

    CHECK(arr.insert(TEST_SIZE-1, inserted, 5) == 5);
    for (int i = 0; i < arr.length(); i++) {
        //printf("%d, ", arr[i]);
        CHECK(arr[i] == ground_truth_1[i]);
    }

    CHECK(arr.insert(0, inserted, 5) == 5);
    for (int i = 0; i < arr.length(); i++) {
        //printf("%d, ", arr[i]);
        CHECK(arr[i] == ground_truth_2[i]);
    }
}

//TEST(Sanity) {
//    CHECK_EQUAL(1, 1);
//}

/**
 * Main
 */
int main(int argc, char * argv[]) {

    return UnitTest::RunAllTests();
}
