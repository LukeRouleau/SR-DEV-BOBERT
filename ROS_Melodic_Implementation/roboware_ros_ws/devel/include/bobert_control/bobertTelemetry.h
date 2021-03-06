// Generated by gencpp from file bobert_control/bobertTelemetry.msg
// DO NOT EDIT!


#ifndef BOBERT_CONTROL_MESSAGE_BOBERTTELEMETRY_H
#define BOBERT_CONTROL_MESSAGE_BOBERTTELEMETRY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bobert_control
{
template <class ContainerAllocator>
struct bobertTelemetry_
{
  typedef bobertTelemetry_<ContainerAllocator> Type;

  bobertTelemetry_()
    : angle()  {
      angle.assign(0.0);
  }
  bobertTelemetry_(const ContainerAllocator& _alloc)
    : angle()  {
  (void)_alloc;
      angle.assign(0.0);
  }



   typedef boost::array<float, 6>  _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::bobert_control::bobertTelemetry_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bobert_control::bobertTelemetry_<ContainerAllocator> const> ConstPtr;

}; // struct bobertTelemetry_

typedef ::bobert_control::bobertTelemetry_<std::allocator<void> > bobertTelemetry;

typedef boost::shared_ptr< ::bobert_control::bobertTelemetry > bobertTelemetryPtr;
typedef boost::shared_ptr< ::bobert_control::bobertTelemetry const> bobertTelemetryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bobert_control::bobertTelemetry_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bobert_control::bobertTelemetry_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bobert_control::bobertTelemetry_<ContainerAllocator1> & lhs, const ::bobert_control::bobertTelemetry_<ContainerAllocator2> & rhs)
{
  return lhs.angle == rhs.angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bobert_control::bobertTelemetry_<ContainerAllocator1> & lhs, const ::bobert_control::bobertTelemetry_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bobert_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bobert_control::bobertTelemetry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bobert_control::bobertTelemetry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bobert_control::bobertTelemetry_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d5646d2d9986672237331f3ea363f45f";
  }

  static const char* value(const ::bobert_control::bobertTelemetry_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd5646d2d99866722ULL;
  static const uint64_t static_value2 = 0x37331f3ea363f45fULL;
};

template<class ContainerAllocator>
struct DataType< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bobert_control/bobertTelemetry";
  }

  static const char* value(const ::bobert_control::bobertTelemetry_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[6] angle # degrees\n"
;
  }

  static const char* value(const ::bobert_control::bobertTelemetry_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct bobertTelemetry_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bobert_control::bobertTelemetry_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bobert_control::bobertTelemetry_<ContainerAllocator>& v)
  {
    s << indent << "angle[]" << std::endl;
    for (size_t i = 0; i < v.angle.size(); ++i)
    {
      s << indent << "  angle[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.angle[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BOBERT_CONTROL_MESSAGE_BOBERTTELEMETRY_H
