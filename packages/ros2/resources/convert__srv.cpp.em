// generated from soss/packages/ros2/resources/convert__srv.cpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating soss/rosidl/ros2/<package>/src/srv/convert__srv__<srv>.cpp files
@#
@# Context:
@#  - spec (rosidl_parser.ServiceSpecification)
@#    Parsed specification of the .srv file
@#  - get_header_filename_from_msg_name (function)
@#######################################################################

@{
camelcase_srv_type = spec.srv_name
underscore_srv_type = get_header_filename_from_msg_name(camelcase_srv_type)

cpp_srv_type = '{}::srv::{}'.format(spec.pkg_name, camelcase_srv_type)

srv_type_string = '{}/{}'.format(spec.pkg_name, camelcase_srv_type)

namespace_parts = [
    'convert', spec.pkg_name, 'srv', underscore_srv_type]
namespace_variable = '__'.join(namespace_parts)

ros2_srv_dependency = '{}/srv/{}.hpp'.format(
      spec.pkg_name, underscore_srv_type)

conversion_dependencies = {}
for type, msg in {"request": spec.request, "response": spec.response}.items():
    for field in msg.fields:
        if field.type.is_primitive_type():
          continue

        key = 'soss/rosidl/ros2/{}/msg/convert__msg__{}.hpp'.format(
            field.type.pkg_name, field.type.type)
        if key not in conversion_dependencies:
            conversion_dependencies[key] = set([])
        conversion_dependencies[key].add(type+'::'+field.name)

alphabetical_request_fields = sorted(spec.request.fields, key=lambda x: x.name)
alphabetical_response_fields = sorted(spec.response.fields, key=lambda x: x.name)
}@

// Include the header for the generic message type
#include <soss/utilities.hpp>

// Include the header for the concrete service type
#include <@(ros2_srv_dependency)>

// Include the headers for the soss message dependencies
@[for key in sorted(conversion_dependencies.keys())]@
#include <@(key)> // @(', '.join(conversion_dependencies[key]))
@[end for]@

// Include the Factory header so we can add this message type to the Factory
#include <soss/ros2/Factory.hpp>

// Include the Node API so we can provide and request services
#include <rclcpp/node.hpp>

namespace soss {
namespace ros2 {
namespace @(namespace_variable) {

using Ros2_Srv = @(cpp_srv_type);
using Ros2_Request = Ros2_Srv::Request;
using Ros2_Response = Ros2_Srv::Response;
const std::string g_srv_name = "@(srv_type_string)";
const std::string g_request_name = g_srv_name + ":request";
const std::string g_response_name = g_srv_name + ":response";

namespace {

//==============================================================================
soss::Message initialize_request()
{
  soss::Message msg;
  msg.type = g_request_name;
@[for field in alphabetical_request_fields]@
  soss::Convert<Ros2_Request::_@(field.name)_type>::add_field(msg, "@(field.name)");
@[end for]@

  return msg;
}

//==============================================================================
void request_to_ros2(const soss::Message& from, Ros2_Request& to)
{
  auto from_field = from.data.begin();
@[for field in alphabetical_request_fields]@
  soss::Convert<Ros2_Request::_@(field.name)_type>::from_soss_field(from_field++, to.@(field.name));
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
  (void)from_field;
}

//==============================================================================
void request_to_soss(const Ros2_Request& from, soss::Message& to)
{
  auto to_field = to.data.begin();
@[for field in alphabetical_request_fields]@
  soss::Convert<Ros2_Request::_@(field.name)_type>::to_soss_field(from.@(field.name), to_field++);
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
  (void)to_field;
}

//==============================================================================
soss::Message initialize_response()
{
  soss::Message msg;
  msg.type = g_response_name;
@[for field in alphabetical_response_fields]@
  soss::Convert<Ros2_Response::_@(field.name)_type>::add_field(msg, "@(field.name)");
@[end for]@

  return msg;
}

//==============================================================================
void response_to_ros2(const soss::Message& from, Ros2_Response& to)
{
  auto from_field = from.data.begin();
@[for field in alphabetical_response_fields]@
  soss::Convert<Ros2_Response::_@(field.name)_type>::from_soss_field(from_field++, to.@(field.name));
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
  (void)from_field;
}

//==============================================================================
void response_to_soss(const Ros2_Response& from, soss::Message& to)
{
  auto to_field = to.data.begin();
@[for field in alphabetical_response_fields]@
  soss::Convert<Ros2_Response::_@(field.name)_type>::to_soss_field(from.@(field.name), to_field++);
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
  (void)to_field;
}

} // anonymous namespace

//==============================================================================
class ClientProxy final : public virtual soss::ServiceClient
{
public:

  ClientProxy(
      rclcpp::Node& node,
      const std::string& service_name,
      const ServiceClientSystem::RequestCallback& callback,
      const rmw_qos_profile_t& qos_profile)
    : _callback(callback),
      _handle(std::make_shared<PromiseHolder>())
  {
    _request = initialize_request();

    _service = node.create_service<Ros2_Srv>(
          service_name,
          [=](const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<Ros2_Request> request,
              const std::shared_ptr<Ros2_Response> response)
              { this->service_callback(request_header, request, response); },
          qos_profile);
  }

  void receive_response(
      std::shared_ptr<void> call_handle,
      const Message& result) override
  {
    const std::shared_ptr<PromiseHolder>& handle =
        std::static_pointer_cast<PromiseHolder>(call_handle);

    response_to_ros2(result, _response);
    handle->promise->set_value(_response);
  }

private:

  void service_callback(
      const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
      const std::shared_ptr<Ros2_Request>& request,
      const std::shared_ptr<Ros2_Response>& response)
  {
    request_to_soss(*request, _request);

    std::promise<Ros2_Response> response_promise;
    _handle->promise = &response_promise;

    std::future<Ros2_Response> future_response = response_promise.get_future();
    _callback(_request, *this, _handle);

    future_response.wait();

    *response = future_response.get();
  }

  struct PromiseHolder
  {
    std::promise<Ros2_Response>* promise;
  };

  const ServiceClientSystem::RequestCallback _callback;
  const std::shared_ptr<PromiseHolder> _handle;
  soss::Message _request;
  Ros2_Response _response;
  rclcpp::Service<Ros2_Srv>::SharedPtr _service;

};

//==============================================================================
std::shared_ptr<soss::ServiceClient> make_client(
    rclcpp::Node& node,
    const std::string& service_name,
    const ServiceClientSystem::RequestCallback& callback,
    const rmw_qos_profile_t& qos_profile)
{
  return std::make_shared<ClientProxy>(node, service_name, callback, qos_profile);
}

namespace {
ServiceClientFactoryRegistrar register_client(g_srv_name, &make_client);
} // anonymous namespace

//==============================================================================
class ServerProxy final : public virtual soss::ServiceProvider
{
public:

  ServerProxy(
      rclcpp::Node& node,
      const std::string& service_name,
      const rmw_qos_profile_t& qos_profile)
    : _service_name(service_name),
      _request_pool(1),
      _response_pool(1)
  {
    _ros2_client = node.create_client<Ros2_Srv>(service_name, qos_profile);
  }

  void call_service(
      const Message& request,
      ServiceClient& soss_client,
      std::shared_ptr<void> call_handle) override
  {
    // This helps the lambda to value-capture the address of the soss client.
    // TODO(MXG): Would it be dangerous for the lambda to reference-capture the
    // soss client? The lambda might be called after this reference has left
    // scope, so when a lambda does a reference-capture of a reference, does it
    // require the reference to stay alive or does it only require the
    // referred-to object to stay alive? For now we'll use this value-capture
    // technique since it's sure to be safe.
    ServiceClient* const ptr_to_soss_client = &soss_client;

    Ros2_Request::SharedPtr ros2_request = _request_pool.pop();
    request_to_ros2(request, *ros2_request);
    _ros2_client->async_send_request(
          ros2_request,
          [=](const rclcpp::Client<Ros2_Srv>::SharedFuture future_response)
          { this->_wait_for_response(*ptr_to_soss_client, std::move(call_handle), future_response, ros2_request); });
  }

private:

  void _wait_for_response(
      ServiceClient& soss_client,
      std::shared_ptr<void> call_handle,
      const rclcpp::Client<Ros2_Srv>::SharedFuture& future_response,
      Ros2_Request::SharedPtr used_request)
  {
    future_response.wait();

    const Ros2_Response::SharedPtr& response = future_response.get();
    soss::Message soss_response = _response_pool.pop();
    response_to_soss(*response, soss_response);

    soss_client.receive_response(std::move(call_handle), soss_response);

    _response_pool.recycle(std::move(soss_response));
    _request_pool.recycle(std::move(used_request));
  }

  const std::string _service_name;
  soss::SharedResourcePool<Ros2_Request> _request_pool;
  soss::ResourcePool<soss::Message, &initialize_response> _response_pool;
  rclcpp::Client<Ros2_Srv>::SharedPtr _ros2_client;

};

//==============================================================================
std::shared_ptr<soss::ServiceProvider> make_server(
    rclcpp::Node& node,
    const std::string& service_name,
    const rmw_qos_profile_t& qos_profile)
{
  return std::make_shared<ServerProxy>(node, service_name, qos_profile);
}

namespace {
ServiceProviderFactoryRegistrar register_server(g_srv_name, &make_server);
}

} // namespace @(namespace_variable)
} // namespace ros2
} // namespace soss
