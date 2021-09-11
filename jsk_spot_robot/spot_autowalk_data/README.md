# spot_autowalk_data

This package includes autowalk data downloaded from public Googledrive folder with jsk_data.

## scripts

### download_autowalk_data.py

this scripts download autowalk data from google drive. this script will be run when catkin build.

### view_map.py

this script visualize autowalk data and each waypoint id

#### Usage

when you want to visualize autowalk data without waypoint id, just run

```
rosrun spot_autowalk_data view_map.py <map_directory.walk>
```

when you want to visualize autowalk data with waypoint id, run the script with `--draw-id` option

```
rosrun spot_autowalk_data view_map.py <map_directory.walk> --draw-id
```

## autowalk data in this packages

### eng2_2FElevator_to_2FEntrance.walk

![eng2_2FElevator_to_2FEntrance](https://user-images.githubusercontent.com/9410362/124133634-fbe92700-dabc-11eb-90d5-84b434729233.png)

### eng2_2FElevator_to_MainGate

![eng2_2FElevator_to_MainGate](https://user-images.githubusercontent.com/9410362/124133994-55e9ec80-dabd-11eb-8ba1-1bd118146244.png)

### eng2_2Felevator_to_eng8_2FElevator

![eng2_2FElevator_to_eng8_2FElevator](https://user-images.githubusercontent.com/9410362/124134033-600beb00-dabd-11eb-823c-483e75318f8b.png)

### eng2_7FElevator_to_2FElevator

![eng2_73B2_to_2FElevator](https://user-images.githubusercontent.com/9410362/124134079-6f8b3400-dabd-11eb-8c99-4fe616f84eee.png)

### eng2_73B2_to_81C1

![eng2_73B2_to_81C1](https://user-images.githubusercontent.com/9410362/124134118-7b76f600-dabd-11eb-829b-c36f105d7051.png)

### eng2_elevator_7FElevator_to_2FElevator

![eng2_elevator_7FElevator_to_2FElevator](https://user-images.githubusercontent.com/9410362/124134162-86318b00-dabd-11eb-86e3-092fcc5e8719.png)

### eng_TelephoneBox_to_HongoMainGate

![eng_TelephoneBox_to_HongoMainGate](https://user-images.githubusercontent.com/9410362/124134207-90ec2000-dabd-11eb-988b-62ffbf98ca67.png)

### eng2_elevator_3FElevator_to_2FElevator.walk

![eng2_elevator_3FElevator_to_2FElevator walk](https://user-images.githubusercontent.com/9410362/124587250-6077fd80-de92-11eb-8014-2852dfa7f60a.png)

### eng2_3FElevator_to_Mech_Office.walk

![eng2_3FElevator_to_Mech_Office walk](https://user-images.githubusercontent.com/9410362/124587299-6f5eb000-de92-11eb-9427-0ed88e6ca414.png)

### eng2_2FElevator_to_MainEntrance.walk

![eng2_2FElevator_to_MainEntrance](https://user-images.githubusercontent.com/9410362/129509474-c3429db1-6a6f-4076-bbbb-c3fb27bc3167.png)

### eng2_MainEntrance_to_HongoMainGate.walk

![eng2_MainEntrance_to_HongoMainGate](https://user-images.githubusercontent.com/9410362/129509500-1bde2818-66c7-4770-a87d-88c4f9d65ae3.png)

### eng2_MainEntrance_to_HongoMainGate_Daylight.walk

![eng2_MainEntrance_to_HongoMainGate_Daylight](https://user-images.githubusercontent.com/9410362/129997430-620ee1dc-6659-4028-acd7-f80b7539364a.png)

### eng2_2FElevator_to_subway.walk

![eng2_2FElevator_to_subway](https://user-images.githubusercontent.com/9410362/132933793-63c627bc-e210-4922-9fa2-4c1538b6f45e.png)

### eng2_subway_tablenavigation.walk

![eng2_subway_tablenavigation](https://user-images.githubusercontent.com/9410362/132933800-6f68566e-8471-4ac4-a030-94414f4a1b83.png)

### eng2_subway_tablenavigation_02.walk

![eng2_subway_tablenavigation_02](https://user-images.githubusercontent.com/9410362/132933805-f3c10dc3-01da-429c-96b8-74225401d703.png)

## How to add autowalk data in this package

1. record a new autowalk data
2. archive and compress an autowalk data directory (e.g. `autowalk.walk`) to `autowalk.walk.tar.gz` by `tar -zcvf autowalk.walk.tar.gz autowalk.walk`
3. upload `autowalk.walk.tar.gz` to [jsk_data public google drive folger](https://drive.google.com/drive/u/0/folders/0B9P1L--7Wd2vUGplQkVLTFBWcFE?resourcekey=0-qlPyB_oRQm887pgRGiPhgg)
4. add new entry to download_autowalk_data.py like

```
    download_data(
        pkg_name=PKG,
        path='autowalk/autowalk.walk.tar.gz',
        url='https://drive.google.com/uc?id=<File ID in Google Drive>',
        md5='<md5 check sum of autowalk.walk.tar.gz>',
        extract=True
    )
```

5. update README.md
