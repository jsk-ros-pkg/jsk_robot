# spot_basic_behaviors

This package includes exaple implementation of base_behavior for spot_behavior_manager.

## walk behavior

This behavior enables Spot to walk a specified route in a autowalk data from a given waypoint to another waypoint.

behavior name: `spot_basic_behaviors.walk_behavior.WalkBehavior`

https://user-images.githubusercontent.com/9410362/124337233-896f6880-dbdc-11eb-9588-30d4a5193bbd.mp4

### Required Configuration

Before using this behavior, belows are required.

- autowalk data containing a route for the behavior
- edge and nodes configuration for the behavior

#### Edge

Fields below are required in args of a edge

- `graph`: path to autowalk data

e.g.

```yaml
- from: 'eng2_73B2'
  to: 'eng2_73A2'
  behavior_type: 'spot_basic_behavior.walk_behavior.WalkBehavior'
  cost: 10
  args:
    graph: '$(find spot_autowalk_data)/autowalk/eng2_73B2_to_7FElevator.walk'
```

#### Start and End Node

Fields below are required for nodes

- `waypoints_on_graph`: list of dict. each dict has `graph`, `id`, `localization_method`
  + `graph`: path to a autowalk data
  + `id`: waypoint id of the start node in the graph
  + `localization_method`: localization_method to be used when starting autowalk, this must be 'fiducial' or 'waypoint'

e.g.

```yaml
  'eng2_73B2':
    waypoints_on_graph:
      - graph: '$(find spot_autowalk_data)/autowalk/eng2_73B2_to_7FElevator.walk'
        id: 'dyed-bat-t00VKo5XixLihCvpsZPRqw=='
        localization_method: 'fiducial'
```

## elevator behavior

This behavior enables Spot to use an elevator to move to anothor floor.

behavior name: `spot_basic_behaviors.elevator_behavior.ElevatorBehavior`

https://user-images.githubusercontent.com/9410362/124337679-cd636d00-dbde-11eb-8db9-c5fedfda4ad9.mp4

### Required Configuration

Before using this behavior, belows are required.

- autowalk data containing a route for the behavior.
- apriltag pose information for elevator door frame detection.
- edge and nodes configuration for the behavior

#### autowalk data

To use this behavior, you need to record autowalk data while Spot riding on the elevator and getting off.

Here is an example.

![124134162-86318b00-dabd-11eb-86e3-092fcc5e8719](https://user-images.githubusercontent.com/9410362/124337765-364ae500-dbdf-11eb-83d3-a99e4025102e.png)

#### apriltag pose information

To use this behavior, an Fiducial (april tag) must be placed on the wall near the elevator doors.
And for elevator door opening and closing detection, transform from elevator foor frame ( center point on the ground ) to the fiducial is required.
Please see https://github.com/sktometometo/jsk_robot/blob/b166ef04cb954b175bedd5653af808be35e42121/jsk_spot_robot/jsk_spot_behaviors/spot_basic_behaviors/config/apriltag/tags.yaml#L44-L54 for examples

#### Edge

Fields below are required in args of a edge

- `graph`: path to autowalk data
- `rest_waypoint_id`: waypoint of rest position in a elevator

e.g.

```yaml
- from: 'eng2_7FElevator'
  to: 'eng2_2FElevator'
  behavior_type: 'spot_basic_behavior.elevator_behavior.ElevatorBehavior'
  cost: 10
  args:
    graph: '$(find spot_autowalk_data)/autowalk/eng2_elevator_7FElevator_to_2FElevator.walk'
    rest_waypoint_id: 'unsaid-collie-jvatvS.7KX9jpzQz61GL4A=='
```

#### Start and End Node

Fields below are required for nodes

- `waypoints_on_graph`: list of dict. each dict has `graph`, `id`, `localization_method`
  + `graph`: path to a autowalk data
  + `id`: waypoint id of the start node in the graph
  + `localization_method`: localization_method to be used when starting autowalk, this must be 'fiducial' or 'waypoint'
- `switchbot_device`: switchbot device name for elevator button

e.g.

```yaml
  'eng2_7FElevator':
    waypoints_on_graph:
      - graph: '$(find spot_autowalk_data)/autowalk/eng2_elevator_7FElevator_to_2FElevator.walk'
        id: 'sly-chetah-IZ4pVY7vrqO36OoKCYk9Zg=='
        localization_method: 'fiducial'
    switchbot_device: '/eng2/7f/elevator/down/button'
```
