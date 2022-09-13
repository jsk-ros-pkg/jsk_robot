# complex_recovery

This package provides recovery behavior plugins which combines multi recoveries to one recovery behavior.
This is useful for assuring a set of recovery behavior to run at one time.

There are two types of recoveries. One is to run multi recoveries sequentially and another is to run them in parallel.

![complex_recovery_diagrams](https://user-images.githubusercontent.com/9410362/189815175-a3265d23-01d4-4ae9-831c-b01aacad2872.png)

## complex_recovery/SequentialComplexRecovery

* `~recovery_behaviors` (list of dictionaries which has `name` and `type` entries, default: None)

## complex_recovery/ParallelComplexRecovery

* `~recovery_behaviors` (list of dictionaries which has `name` and `type` entries, default: None)
