#!/bin/bash

ALDEBARAN_VER=1.14.5
LINUXARCH=64

source  ~/.config/user-dirs.dirs
cd `rospack find naoeus`

CHORE_DIR=${XDG_DOWNLOAD_DIR}/choregraphe-suite-$ALDEBARAN_VER-linux$LINUXARCH
if [ ! -e ${CHORE_DIR} ]; then
    scp $USER@aries.jsk.t.u-tokyo.ac.jp:/home/jsk/eus/archives/Nao/choregraphe-suite-$ALDEBARAN_VER-linux$LINUXARCH.tar.gz ${XDG_DOWNLOAD_DIR}
    tar  -C ${XDG_DOWNLOAD_DIR} -xvzf ${XDG_DOWNLOAD_DIR}/choregraphe-suite-$ALDEBARAN_VER-linux$LINUXARCH.tar.gz
fi

if [ ! -e `rospack find nao_description`/meshes ]; then
    ln -sf  ${CHORE_DIR}/share/alrobotmodel/meshes `rospack find nao_description`/meshes
fi

NAO_DESCRIPTION_PATCH=`pwd`/patch/nao_description.patch
(cd `rospack find nao_description`; patch -N -p0 < ${NAO_DESCRIPTION_PATCH})

which OgreXMLConverter || sudo apt-get install ogre-tools
export LD_LIBRARY_PATH=`rospack find assimp_devel`/lib:$LD_LIBRARY_PATH
export PATH=`rospack find assimp_devel`/bin:$PATH
(cd `rospack find nao_description`/meshes; 
  ln -sf nao.material Scene.material ## enable to read material from assimp
  find . -name '*.mesh' -exec OgreXMLConverter {} \; ## .mesh -> .mesh.xml
  find . -name '*.mesh' -exec assimp_devel export {}.xml {}.dae \; ## ## .mesh.xml -> .dae
  cd nao/V40;
  for png in ../../*.png; do
    ln -sf $png .
  done
  )

(cd `rospack find rviz`/ogre_media;
 sudo ln -sf ${CHORE_DIR}/share/alrobotmodel/meshes/nao.material materials;
 sudo ln -sf ${CHORE_DIR}/share/alrobotmodel/meshes/*.png textures;)

(cd `rospack find euscollada`; ./nao.sh)

cat <<EOF > euslisp/nao.l
(load "package://eus_assimp/euslisp/eus-assimp.l")
(load "package://euscollada/nao.l")
(defun nao ()
  (setq *nao* (instance nao-robot :init))
  (update-to-original-meshfile *nao* :scale 1.0)
  *nao*)
EOF

roseus euslisp/nao.l "(nao)" "(objects (list *nao*))" "(unix:sleep 5)" "(unix::exit 0)"