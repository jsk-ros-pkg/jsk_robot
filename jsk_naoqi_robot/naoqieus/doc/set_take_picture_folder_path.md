## :set-take-picture-folder-path `name` (naoqi_bridge [`kochigami-develop`])

### What is this?

Changes a file path which a photo taken by a robot by using `:take-picture` is stored

As a default, a photo will be stored in `/home/nao/.local/share/PackageManager/apps/img/html/`.

This method will change <folder name> of `/home/nao/.local/share/PackageManager/apps/<folder name>/html/`.

### Parameters

`name`: directory name (str, default is `img`)

### Location

`naoqi_apps/launch/photo_capture.launch`  

### Sample

```
; As a default, 1.jpg will be stored in `/home/nao/.local/share/PackageManager/apps/img/html/`

send *ri* :take-picture "1"

; In this sample, 2.jpg taken by a NAOqi robot will be stored in /home/nao/.local/share/PackageManager/apps/test/html/

send *ri* :set-take-picture-folder-path "test"
send *ri* :take-picture "2"
```

