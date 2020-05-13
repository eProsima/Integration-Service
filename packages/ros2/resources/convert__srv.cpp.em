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

namespace_parts_srv = [
    'convert', spec.pkg_name, 'srv', underscore_srv_type]
namespace_variable_srv = '__'.join(namespace_parts_srv)

namespace_parts_msg = [
    'convert', spec.pkg_name, 'msg', underscore_srv_type]
namespace_variable_msg = '__'.join(namespace_parts_msg)

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

#include <stdexcept>

// Include the header for the generic message type
#include <soss/Message.hpp>

// Include the header for the conversions
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

#include <chrono>

namespace soss {
namespace ros2 {
namespace @(namespace_variable_srv) {

using Ros2_Srv = @(cpp_srv_type);
using Ros2_Request = Ros2_Srv::Request;
using Ros2_Response = Ros2_Srv::Response;
const std::string g_srv_name = "@(srv_type_string)";
const std::string g_request_name = g_srv_name + ":request";
const std::string g_response_name = g_srv_name + ":response";
const std::string g_idl = R"~~~(
@(idl)
)~~~";

namespace {
const xtypes::StructType& request_type() {
  xtypes::idl::Context context;
  context.allow_keyword_identifiers = true;
  context.ignore_redefinition = true;
  xtypes::idl::parse(g_idl, context);
  if (!context.success)
  {
    throw std::runtime_error("Failed while parsing request type @(cpp_srv_type)_Request");
  }
  static xtypes::StructType type(context.module().structure("@(cpp_srv_type)_Request"));
  type.name(g_request_name);
  return type;
}

TypeFactoryRegistrar register_request_type(g_request_name, &request_type);

const xtypes::StructType& response_type() {
  xtypes::idl::Context context;
  context.allow_keyword_identifiers = true;
  context.ignore_redefinition = true;
  xtypes::idl::parse(g_idl, context);
  if (!context.success)
  {
    throw std::runtime_error("Failed while parsing response type @(cpp_srv_type)_Response");
  }
  static xtypes::StructType type(context.module().structure("@(cpp_srv_type)_Response"));
  type.name(g_response_name);
  return type;
}

TypeFactoryRegistrar register_response_type(g_response_name, &response_type);
} // anonymous namespace


//==============================================================================
void request_to_ros2(const xtypes::ReadableDynamicDataRef& from, Ros2_Request& to)
{
@[for field in alphabetical_request_fields]@
  soss::Convert<Ros2_Request::_@(field.name)_type>::from_xtype_field(from["@(field.name)"], to.@(field.name));
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
}

//==============================================================================
void request_to_xtype(const Ros2_Request& from, xtypes::WritableDynamicDataRef to)
{
@[for field in alphabetical_request_fields]@
  soss::Convert<Ros2_Request::_@(field.name)_type>::to_xtype_field(from.@(field.name), to["@(field.name)"]);
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
}

//==============================================================================
void response_to_ros2(const xtypes::ReadableDynamicDataRef& from, Ros2_Response& to)
{
@[for field in alphabetical_response_fields]@
  soss::Convert<Ros2_Response::_@(field.name)_type>::from_xtype_field(from["@(field.name)"], to.@(field.name));
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
}

//==============================================================================
void response_to_xtype(const Ros2_Response& from, xtypes::WritableDynamicDataRef to)
{
@[for field in alphabetical_response_fields]@
  soss::Convert<Ros2_Response::_@(field.name)_type>::to_xtype_field(from.@(field.name), to["@(field.name)"]);
@[end for]@

  // Suppress possible unused variable warnings
  (void)from;
  (void)to;
}

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
      _handle(std::make_shared<PromiseHolder>()),
      _request_data(request_type())
  {
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
      const xtypes::DynamicData& result) override
  {
    const std::shared_ptr<PromiseHolder>& handle =
        std::static_pointer_cast<PromiseHolder>(call_handle);

    response_to_ros2(result, _response);
    handle->promise->set_value(_response);
  }

private:

  void service_callback(
      const std::shared_ptr<rmw_request_id_t>&, //request_header
      const std::shared_ptr<Ros2_Request>& request,
      const std::shared_ptr<Ros2_Response>& response)
  {
    request_to_xtype(*request, _request_data);

    std::promise<Ros2_Response> response_promise;
    _handle->promise = &response_promise;

    std::future<Ros2_Response> future_response = response_promise.get_future();
    _callback(_request_data, *this, _handle);

    if (std::future_status::ready == future_response.wait_for(std::chrono::milliseconds(5000))) // TODO: Make waiting time configurable.
    {
      *response = future_response.get();
    }
    else
    {
      std::cout << "Request timeout." << std::endl;
    }
  }

  struct PromiseHolder
  {
    std::promise<Ros2_Response>* promise;
  };

  const ServiceClientSystem::RequestCallback _callback;
  const std::shared_ptr<PromiseHolder> _handle;
  xtypes::DynamicData _request_data;
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
ServiceClientFactoryRegistrar register_client(g_response_name, &make_client);
} // anonymous namespace

//==============================================================================
xtypes::DynamicData initialize_response() {
  return xtypes::DynamicData(response_type());
}
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
      const xtypes::DynamicData& request,
      ServiceClient& soss_client,
      std::shared_ptr<void> call_handle) override
  {
    if (!_ros2_client->wait_for_service(std::chrono::milliseconds(10)))
    {
      return;
    }

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
    xtypes::DynamicData soss_response = _response_pool.pop();
    response_to_xtype(*response, soss_response);

    soss_client.receive_response(std::move(call_handle), soss_response);

    _response_pool.recycle(std::move(soss_response));
    _request_pool.recycle(std::move(used_request));
  }

  const std::string _service_name;
  soss::SharedResourcePool<Ros2_Request> _request_pool;
  soss::ResourcePool<xtypes::DynamicData, &initialize_response> _response_pool;
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
ServiceProviderFactoryRegistrar register_server(g_request_name, &make_server);
}

} // namespace @(namespace_variable_srv)
} // namespace ros2
} // namespace soss
