#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'spot_autowalk_data'

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_73B2_to_81C1.walk.tar.gz',
        url='https://drive.google.com/uc?id=1IDTCP7n4LCowizW3mFQvTOQtE4qOH9tx',
        md5='65f09629c0ac2aa21df6f179c9875bd0',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_73B2_to_7FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=1O8o6voq2v8WenfaUYcmpSU-IwJSsXW5_',
        md5='ecd4d8dc043995f7675a59fce950676b',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_7FElevator_to_2FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=12MOg5okckmQlYiM6flkdeMYxqjn9-C9l',
        md5='67ae3210cbfb55791fff6494f84abb3b',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_elevator_7FElevator_to_2FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=1iyx0y1dPu4HUPMNepR_VaZd_WEaC5Lku',
        md5='915916d084abd54c2c17f0738a726da3',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_2FElevator_to_2FEntrance.walk.tar.gz',
        url='https://drive.google.com/uc?id=1cYUn_qnRslWuH0ZMEBN6ovqmdcUB0MzY',
        md5='78c6e1e8e5967b216c9f53e38893750e',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_2FElevator_to_MainGate.walk.tar.gz',
        url='https://drive.google.com/uc?id=1CIuStpjxIA188MLxUsfUjI-47Ev36Wiv',
        md5='47d669bcb1394b97c95e5d77f78da3e5',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_2FElevator_to_eng8_2FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=1oyU1ufqy9gryPXw8YZ45Ff7AR8K825dT',
        md5='ac9a67567104df2ddd78b8c53fa61ba2',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng_TelephoneBox_to_HongoMainGate.walk.tar.gz',
        url='https://drive.google.com/uc?id=120WC6SE4C_9XIvy0j3HEraljIigjcJ5U',
        md5='6f89cd74bea3934171e0ae720747da24',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_3FElevator_to_Mech_Office.walk.tar.gz',
        url='https://drive.google.com/uc?id=10pWP1qnbr5TCfQVUeOHnDacdwQ3X3Awh',
        md5='52564afd6471c9551c8f77ac031a366d',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_elevator_3FElevator_to_2FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=1OFB38ISYjTtu9A87QtfYyVLhtcGnpu2a',
        md5='e8ed341b22c5eb66373d29b580bede05',
        extract=True
    )


if __name__ == '__main__':
    main()
