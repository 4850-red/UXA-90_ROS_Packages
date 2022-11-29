ARG ROS_VERSION=humble
ARG ROS_PLATFORM=

FROM ${ROS_PLATFORM}ros:${ROS_VERSION} AS builder
ARG OVERLAY_WS=/opt/ros/overlay_ws

WORKDIR ${OVERLAY_WS}
COPY . .

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --parallel-workers 2



FROM ${ROS_PLATFORM}ros:${ROS_VERSION}-ros-core AS exec
ARG OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR ${OVERLAY_WS}

COPY --from=builder ${OVERLAY_WS}/install .

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/setup.bash"' \
      /ros_entrypoint.sh

# run launch command
CMD ["ros2", "launch", "uxa_serial", "uxa-system-launch.xml"]