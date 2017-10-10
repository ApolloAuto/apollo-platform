#include <pluginlib/class_list_macros.h>
#include <image_transport_tutorial/resized_publisher.h>
#include <image_transport_tutorial/resized_subscriber.h>

PLUGINLIB_REGISTER_CLASS(resized_pub, ResizedPublisher, image_transport::PublisherPlugin)

PLUGINLIB_REGISTER_CLASS(resized_sub, ResizedSubscriber, image_transport::SubscriberPlugin)

