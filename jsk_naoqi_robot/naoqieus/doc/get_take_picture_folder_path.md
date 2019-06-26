## :get-take-picture-folder-path (naoqi_bridge [`kochigami-develop`])

### What is this?

Gets a file path to store a photo taken by a robot by using `:take-picture`.

As a default, photo will be stored in `/home/nao/.local/share/PackageManager/apps/img/html/`.

### Location

`naoqi_apps/launch/photo_capture.launch`

### Sample

```
send *ri* :get-take-picture-folder-path

"/home/nao/.local/share/PackageManager/apps/img/html/"

send *ri* :set-take-picture-folder-path "test"
send *ri* :get-take-picture-folder-path

"/home/nao/.local/share/PackageManager/apps/test/html/"
```