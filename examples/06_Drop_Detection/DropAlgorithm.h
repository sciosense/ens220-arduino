// DropAlgorithm.h
#include <Arduino.h>

#ifndef DROP_ALGORITHM
#define DROP_ALGORITHM

// Definition of the State struct
typedef struct {
    int32_t state;
    int32_t current_mean;
    int32_t amount_previous_baselines;
    int32_t* previous_baselines;
    int32_t drop_event;
    int32_t max_window;
    int32_t min_window;
    int32_t mean_window;
    int32_t index_window;
    int32_t drop_count;
    double last_baseline_update;
    double spike_height;
    double spike_duration;
} DropDetectionState;

typedef struct {
    int32_t min_diff_th;
    int32_t min_spike_th;
    int32_t baseline_max_diff_th;
    double baseline_update_period;
    int32_t window_size;
} AlgorithmConstants;

DropDetectionState* CreateAlgorithmStruct(int32_t amount_previous_baselines);
void DestroyAlgorithmStruct(DropDetectionState* state);
void InitializeAlgorithmStruct(DropDetectionState* state, int32_t initial_value);
void UpdateBaseline(DropDetectionState* state, int32_t new_baseline, unsigned long time);
void ResetBaseline(DropDetectionState* state);
void FindMaxDifferenceWithBaseline(DropDetectionState* state, int32_t *maxValue, int32_t *maxIndex, int32_t *minValue, int32_t *minIndex);
void detect_drop_impact_state(DropDetectionState* state, int32_t p, unsigned long elapsed_time, AlgorithmConstants algorithm_constants);

#endif // DROP_ALGORITHM