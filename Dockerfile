ARG ROS_VERSION=humble

FROM ros:${ROS_VERSION} AS builder
ARG OVERLAY_WS=/opt/ros/overlay_ws

WORKDIR ${OVERLAY_WS}
COPY . .

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build



FROM ros:${ROS_VERSION}-ros-core AS exec
ARG OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR ${OVERLAY_WS}

COPY --from=builder ${OVERLAY_WS}/install .

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/setup.bash"' \
      /ros_entrypoint.sh

RUN echo "chmod 777 /dev/ttyUSB0" >> /cmd.sh && echo "ls -l /dev" >> /cmd.sh && echo "ros2 launch uxa_serial uxa-system-launch.xml" >> /cmd.sh
# run launch file
CMD ["sh", "/cmd.sh"]