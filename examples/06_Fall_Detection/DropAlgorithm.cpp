#include "DropAlgorithm.h"

DropDetectionState* CreateAlgorithmStruct(int32_t amount_previous_baselines) {
    DropDetectionState* state = (DropDetectionState*)malloc(sizeof(DropDetectionState));
    if (state == NULL) {
        return NULL; // Allocation failed
    }
    state->amount_previous_baselines = amount_previous_baselines;
    state->previous_baselines = (int32_t*)malloc(amount_previous_baselines * sizeof(int32_t));
    if (state->previous_baselines == NULL) {
        free(state);
        return NULL; // Allocation failed
    }
    return state;
}

void DestroyAlgorithmStruct(DropDetectionState* state) {
    // Free the memory allocated for the struct
    if (state != NULL) {
        free(state->previous_baselines);
        free(state);
    }
}

void InitializeAlgorithmStruct(DropDetectionState* state, int32_t initial_value) {
    state->state = 0;
    state->drop_event = 0;
    state->current_mean = initial_value;
    state->max_window = initial_value;
    state->min_window = initial_value;
    state->mean_window = initial_value;
    state->index_window=0;
    state->spike_height=0;
    state->drop_count=0;
    state->last_baseline_update=0;
    ResetBaseline(state);
}

void UpdateBaseline(DropDetectionState* state, unsigned long time, double update_period) {
    if (time - state->last_baseline_update > update_period) {
        state->last_baseline_update = time;
        // update the previous baselines with the new mean
        for (int i=state->amount_previous_baselines; i>0; i--) {
            state->previous_baselines[i] = state->previous_baselines[i-1];
        }
        state->previous_baselines[0] = state->current_mean;
    }
    // update the current pressure for height calculation
    state->current_mean = state->mean_window;
}

void ResetBaseline(DropDetectionState* state) {
    for (int i = 0; i < state->amount_previous_baselines; ++i) {
        state->previous_baselines[i] = state->current_mean;
    }
}

void FindMaxDifferenceWithBaseline(DropDetectionState* state, int32_t *maxValue, int32_t *maxIndex, int32_t *minValue, int32_t *minIndex) {
    *maxValue = INT32_MIN;
    *minValue = INT32_MAX;
    *maxIndex = 0;
    *minIndex = 0;
    for (int i = 0; i < state->amount_previous_baselines; i++) {
        int32_t difference_with_baseline = state->current_mean - state->previous_baselines[i];
        if (difference_with_baseline > *maxValue) {
            *maxValue = difference_with_baseline;
            *maxIndex = i;
        }
        if (difference_with_baseline < *minValue) {
            *minValue = difference_with_baseline;
            *minIndex = i;
        }
    }
}

void detect_drop_impact_state(DropDetectionState* state, int32_t new_value, unsigned long elapsed_time, AlgorithmConstants algorithm_constants) {
    state->drop_event = 0;
    state->spike_height = 0;
    state->spike_duration = 0;
    
    if (state->index_window == 0) {
        // initilize variables at the start of the window
        state->max_window = new_value;
        state->min_window = new_value;
        state->mean_window = new_value;
    } else {
        if (new_value > state->max_window) { 
            state->max_window = new_value;
        }
        if (new_value < state->min_window) {
            state->min_window = new_value;
        }
        //update sum and later divide by window size
        state->mean_window += new_value;
        
    }
    state->index_window += 1;

    if (state->index_window == algorithm_constants.window_size) {
        // end of the window
        state->mean_window = state->mean_window / algorithm_constants.window_size; 
        state->index_window = 0;

        int32_t range_window = state->max_window - state->min_window;
        if (range_window < algorithm_constants.baseline_max_diff_th) {
            // Add a new baseline in order to calculate the relative values later
            UpdateBaseline(state, elapsed_time, algorithm_constants.baseline_update_period);
        }

        if (state->state == 0) { 
            // state: No spike yet
            if (range_window >= algorithm_constants.min_spike_th) {
                // Spike detected
                state->state = 1;
            }
        } else {
            // The spike state will be completed once the values present again a small variation in the window
            if (range_window < algorithm_constants.baseline_max_diff_th) {
                state->state = 0;
                int32_t max_relative_difference = 0;
                int32_t max_index;
                int32_t min_relative_difference = 0;
                int32_t min_index;
                FindMaxDifferenceWithBaseline(state, &max_relative_difference, &max_index, &min_relative_difference, &min_index);
                if (max_relative_difference > algorithm_constants.min_diff_th) { 
                    // if the change in height is larger than a certain threshold then it was a fall plus impact
                    state->drop_event = 1;
                    state->drop_count += 1;
                    state->spike_height = max_relative_difference;
                    state->spike_duration = max_index;
                    // Prevent a second drop from being detected immediately after the drop
                    ResetBaseline(state);
                }
                else if(min_relative_difference < -algorithm_constants.min_diff_th) {
                    // if the change in height is larger than a certain threshold then it was a fall plus impact
                    state->drop_event = 1;
                    state->drop_count += 1;
                    state->spike_height = -min_relative_difference;
                    state->spike_duration = min_index;
                    // Prevent a second drop from being detected immediately after the drop
                    ResetBaseline(state);
                }
            }
        }
    }
}


