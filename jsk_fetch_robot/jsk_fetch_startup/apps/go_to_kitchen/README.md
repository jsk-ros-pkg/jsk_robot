go_to_kitchen
====================

## Overview


<!--https://user-images.githubusercontent.com/67531577/178182693-c60ee282-a128-4b96-9ec4-6dc318d8a235.mp4-->
![FetchKitchenDemo](https://user-images.githubusercontent.com/67531577/178187049-762d6351-4589-466f-9884-1bb82cd38bef.gif)


## Usage

There are three different demo launch methods

- App
  - Add fetch to `rwt_app_chooser`
    https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_fetch_robot/jsk_fetch_startup#apps
  - Select `Go to kitchen` demo

    ![FetchKitchenApp](https://user-images.githubusercontent.com/19769486/178192990-36be7efb-178f-447b-9df7-74173d844682.png)

- Voice call
  - Conversation example

    User 「ねぇねぇ」

    Fetch 「はい、なんでしょう」

    User 「キッチンに行ってきて」

    Fetch 「キッチンに行きます」

  - For detail, please see [dialogflow_task_executive](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/dialogflow_task_executive)

- Command line
  - Use app_manager service call
  - `rossermaster fetch1075`
  - `rosservice call /fetch1075/start_app "name: 'jsk_fetch_startup/go_to_kitchen'`

## Citation
Coming soon!
