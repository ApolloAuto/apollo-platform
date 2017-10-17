#include <image_transport/simple_subscriber_plugin.h>
#include <image_transport_tutorial/ResizedImage.h>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<image_transport_tutorial::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void internalCallback(const typename image_transport_tutorial::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
};
