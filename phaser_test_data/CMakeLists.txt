cmake_minimum_required (VERSION 2.8)
project(phaser_test_data)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

install(DIRECTORY rotation_and_translation
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY translation_only
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rotation_only
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
cs_install()
cs_export(CFG_EXTRAS phaser-test-data.cmake phaser-import-test-maps.cmake)
