/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Include the header for the generic message type
#include <soss/utilities.hpp>

// Include the header for the concrete service type
#include <nav_msgs/srv/get_plan.hpp>

// Include the headers for the soss message dependencies
#include "soss__ros2__convert__nav_msgs__msg__path.hpp"

// Include the Factory header so we can add this message type to the Factory
#include <soss/ros2/Factory.hpp>

// Include the Node API so we can provide and request services
#include <rclcpp/node.hpp>

namespace soss {
namespace ros2 {
namespace convert__nav_msgs__srv__get_plan {

using Ros2_Srv = nav_msgs::srv::GetPlan;
using Ros2_Request = Ros2_Srv::Request;
using Ros2_Response = Ros2_Srv::Response;
const std::string g_srv_name = "nav_msgs/GetPlan";
const std::string g_request_name = g_srv_name + ":request";
const std::string g_response_name = g_srv_name + ":response";

namespace {

//==============================================================================
soss::Message initialize_request()
{
  soss::Message msg;
  msg.type = g_request_name;
  soss::Convert<Ros2_Request::_goal_type>::add_field(msg, "goal");
  soss::Convert<Ros2_Request::_start_type>::add_field(msg, "start");
  soss::Convert<Ros2_Request::_tolerance_type>::add_field(msg, "tolerance");

  return msg;
}

//==============================================================================
void request_to_ros2(const soss::Message& from, Ros2_Request& to)
{
  auto it = from.data.begin();
  soss::Convert<Ros2_Request::_goal_type>::from_soss_field(it++, to.goal);
  soss::Convert<Ros2_Request::_start_type>::from_soss_field(it++, to.start);
  soss::Convert<Ros2_Request::_tolerance_type>::from_soss_field(it++, to.tolerance);
}

//==============================================================================
void request_to_soss(const Ros2_Request& from, soss::Message& to)
{
  auto it = to.data.begin();
  soss::Convert<Ros2_Request::_goal_type>::to_soss_field(from.goal, it++);
  soss::Convert<Ros2_Request::_start_type>::to_soss_field(from.start, it++);
  soss::Convert<Ros2_Request::_tolerance_type>::to_soss_field(from.tolerance, it++);
}

//==============================================================================
soss::Message initialize_response()
{
  soss::Message msg;
  msg.type = g_response_name;
  soss::Convert<Ros2_Response::_plan_type>::add_field(msg, "plan");

  return msg;
}

//==============================================================================
void response_to_ros2(const soss::Message& from, Ros2_Response& to)
{
  auto it = from.data.begin();
  soss::Convert<Ros2_Response::_plan_type>::from_soss_field(it++, to.plan);
}

//==============================================================================
void response_to_soss(const Ros2_Response& from, soss::Message& to)
{
  auto it = to.data.begin();
  soss::Convert<Ros2_Response::_plan_type>::to_soss_field(from.plan, it++);
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
      const std::shared_ptr<void>& call_handle,
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
      const std::shared_ptr<void>& call_handle) override
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
          { this->_wait_for_response(*ptr_to_soss_client, call_handle, future_response, ros2_request); });
  }

private:

  void _wait_for_response(
      ServiceClient& soss_client,
      const std::shared_ptr<void>& call_handle,
      const rclcpp::Client<Ros2_Srv>::SharedFuture& future_response,
      Ros2_Request::SharedPtr used_request)
  {
    future_response.wait();

    const Ros2_Response::SharedPtr& response = future_response.get();
    soss::Message soss_response = _response_pool.pop();
    response_to_soss(*response, soss_response);

    soss_client.receive_response(call_handle, soss_response);

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

} // namespace convert__nav_msgs__srv__get_plan
} // namespace ros2
} // namespace soss
