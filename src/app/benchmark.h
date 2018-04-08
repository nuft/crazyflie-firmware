#ifndef BENCHMARK_H
#define BENCHMARK_H

#ifdef __cplusplus
extern "C" {
#endif

void cycle_counter_reset(void);
uint32_t cycle_counter_get(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCHMARK_H */
