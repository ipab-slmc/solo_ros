# Build:
# $ docker build --rm -f Dockerfile -t jshim/solo .
#
# Run:
# $ rocker --x11 --nvidia --name solo jshim/solo
#
# Pull:
# $ docker pull jshim/solo

# FROM osrf/ros:noetic-desktop
FROM dorowu/ubuntu-desktop-lxde-vnc:focal
MAINTAINER sjh2808@gmail.com

# TODO(JaehyunShim): Write
RUN apt-get update
