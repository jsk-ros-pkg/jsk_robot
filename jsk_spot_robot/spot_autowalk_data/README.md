# spot_autowalk_data

This package includes autowalk data downloaded from public Googledrive folder with jsk_data.

## scripts

### download_autowalk_data.py

this scripts download autowalk data from google drive. this script will be run when catkin build.

### view_map.py

this script visualize autowalk data and each waypoint id

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

### eng_elevator_3FElevator_to_2FElevator.walk

![eng2_elevator_3FElevator_to_2FElevator walk](https://user-images.githubusercontent.com/9410362/124587250-6077fd80-de92-11eb-8014-2852dfa7f60a.png)

### eng_3FElevator_to_Mech_Office.walk

![eng2_3FElevator_to_Mech_Office walk](https://user-images.githubusercontent.com/9410362/124587299-6f5eb000-de92-11eb-9427-0ed88e6ca414.png)

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
