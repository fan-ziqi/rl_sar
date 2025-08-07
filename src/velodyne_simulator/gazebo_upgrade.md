# GPU issues
The GPU problems reported in this [issue](https://bitbucket.org/osrf/gazebo/issues/946/) have been solved with this [pull request](https://bitbucket.org/osrf/gazebo/pull-requests/2955/) for the ```gazebo7``` branch. The Gazebo versions from the ROS apt repository (7.0.0 for Kinetic, 9.0.0 for Melodic) do not have this fix. One solution is to pull up-to-date packages from the OSRF Gazebo apt repository. Another solution is to compile from source: http://gazebosim.org/tutorials?tut=install_from_source

* The GPU fix was added to ```gazebo7``` in version 7.14.0
* The GPU fix was added to ```gazebo9``` in version 9.4.0

# Use up-to-date packages from the OSRF Gazebo apt repository
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sudo apt update
sudo apt upgrade
```

