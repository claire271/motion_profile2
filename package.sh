#!/bin/sh
cd build
jar cvfe motion_profile2.jar com.tigerhuang.motion_profile2.MainFrame `find . -name "*.class"`
