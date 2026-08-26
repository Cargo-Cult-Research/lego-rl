// Host-side test driver for pybricks/experimental/pb_mlp.c.
//
// Reads one network and a batch of test inputs from stdin as text, evaluates
// each input, and prints the outputs. test_mlp_kernel.py compiles this
// against the pybricks-micropython checkout and compares the results with a
// NumPy reference implementation.
//
// Input format (whitespace separated):
//   num_sizes  sizes...  tanh_output  params...  num_tests  inputs...

#include <stdio.h>
#include <stdlib.h>

#include <pybricks/experimental/pb_mlp.h>

int main(void) {
    size_t num_sizes;
    if (scanf("%zu", &num_sizes) != 1 || num_sizes < 2 || num_sizes > PB_MLP_MAX_NUM_SIZES) {
        return 1;
    }

    uint16_t sizes[PB_MLP_MAX_NUM_SIZES];
    for (size_t i = 0; i < num_sizes; i++) {
        unsigned int size;
        if (scanf("%u", &size) != 1) {
            return 1;
        }
        sizes[i] = size;
    }

    int tanh_output;
    if (scanf("%d", &tanh_output) != 1) {
        return 1;
    }

    size_t num_params = pb_mlp_num_params(sizes, num_sizes);
    float *params = malloc(num_params * sizeof(float));
    for (size_t i = 0; i < num_params; i++) {
        if (scanf("%f", &params[i]) != 1) {
            return 1;
        }
    }

    size_t max_width = pb_mlp_max_width(sizes, num_sizes);
    float *input = malloc(max_width * sizeof(float));
    float *output = malloc(max_width * sizeof(float));
    float *scratch = malloc(2 * max_width * sizeof(float));

    size_t num_tests;
    if (scanf("%zu", &num_tests) != 1) {
        return 1;
    }

    for (size_t t = 0; t < num_tests; t++) {
        for (size_t i = 0; i < sizes[0]; i++) {
            if (scanf("%f", &input[i]) != 1) {
                return 1;
            }
        }
        pb_mlp_forward(sizes, num_sizes, params, tanh_output, input, output, scratch);
        for (size_t i = 0; i < sizes[num_sizes - 1]; i++) {
            printf("%.9g ", (double)output[i]);
        }
        printf("\n");
    }

    return 0;
}
