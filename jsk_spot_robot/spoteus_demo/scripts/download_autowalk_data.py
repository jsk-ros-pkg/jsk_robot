#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'spoteus_demo'

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_73b2_to_81c1_night.walk.tar.gz',
        url='https://drive.google.com/uc?id=1PXgMsmN3uKPG3YAap9vdfrBduEiLgAl8',
        md5='a79dd142013b12c0babb4400be583739',
        extract=True
    )


if __name__ == '__main__':
    main()
