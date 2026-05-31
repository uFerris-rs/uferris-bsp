
| Option | Stability | Default&nbsp;value | Allowed&nbsp;values |
|--------|:---------:|:------------------:|:-------------------:|
| <p>**ESP_RTOS_CONFIG_TICK_RATE_HZ**</p> <p>Tick rate of the task scheduler in Hertz</p> | ⚠️ Unstable | 100 | Positive integer
| <p>**ESP_RTOS_CONFIG_SW_TASK_OVERFLOW_DETECTION**</p> <p>Enable software-based stack overflow detection. The stack guard value and offset is based on esp-hal configuration.</p> | ⚠️ Unstable | false | 
| <p>**ESP_RTOS_CONFIG_STACK_POINTER_RANGE_CHECK**</p> <p>Enable range-check based stack overflow detection.</p> | ⚠️ Unstable | true | 
| <p>**ESP_RTOS_CONFIG_HW_TASK_OVERFLOW_DETECTION**</p> <p>Enable hardware-based stack overflow detection. The stack watermark is based on the esp-hal stack-guard-offset configuration.</p> | ⚠️ Unstable | true | 
