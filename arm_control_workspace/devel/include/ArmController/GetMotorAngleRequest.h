// Generated by gencpp from file ArmController/GetMotorAngleRequest.msg
// DO NOT EDIT!


#ifndef ARMCONTROLLER_MESSAGE_GETMOTORANGLEREQUEST_H
#define ARMCONTROLLER_MESSAGE_GETMOTORANGLEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ArmController
{
template <class ContainerAllocator>
struct GetMotorAngleRequest_
{
  typedef GetMotorAngleRequest_<ContainerAllocator> Type;

  GetMotorAngleRequest_()
    : motorID(0)  {
    }
  GetMotorAngleRequest_(const ContainerAllocator& _alloc)
    : motorID(0)  {
    }



   typedef int8_t _motorID_type;
  _motorID_type motorID;




  typedef boost::shared_ptr< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetMotorAngleRequest_

typedef ::ArmController::GetMotorAngleRequest_<std::allocator<void> > GetMotorAngleRequest;

typedef boost::shared_ptr< ::ArmController::GetMotorAngleRequest > GetMotorAngleRequestPtr;
typedef boost::shared_ptr< ::ArmController::GetMotorAngleRequest const> GetMotorAngleRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ArmController::GetMotorAngleRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ArmController

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "43c35bf09924a653714ffdbe59d23676";
  }

  static const char* value(const ::ArmController::GetMotorAngleRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x43c35bf09924a653ULL;
  static const uint64_t static_value2 = 0x714ffdbe59d23676ULL;
};

template<class ContainerAllocator>
struct DataType< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ArmController/GetMotorAngleRequest";
  }

  static const char* value(const ::ArmController::GetMotorAngleRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 motorID\n\
";
  }

  static const char* value(const ::ArmController::GetMotorAngleRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.motorID);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GetMotorAngleRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ArmController::GetMotorAngleRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ArmController::GetMotorAngleRequest_<ContainerAllocator>& v)
  {
    s << indent << "motorID: ";
    Printer<int8_t>::stream(s, indent + "  ", v.motorID);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARMCONTROLLER_MESSAGE_GETMOTORANGLEREQUEST_H
