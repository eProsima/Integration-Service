# SOSS with XTypes



## Introduction
1. XTypes API
1. System Handle interface changes
1. YAML changes
1. Future work



## Xtypes API


### DynamicType definition
```c++
StructType inner("InnerType");
inner.add_member(Member("m_uint", primitive_type<uint32_t>()));
inner.add_member(Member("m_float", primitive_type<float>()).optional());

StructType outter("OutterType");
outter.add_member("double", primitive_type<double>());
outter.add_member("inner", inner);
outter.add_member("string", StringType());
outter.add_member("seq_uint", SequenceType(primitive_type<uint32_t>(), 5));
outter.add_member("seq_inner", SequenceType(inner));
outter.add_member("array_uint", ArrayType(primitive_type<uint32_t>(), 4));
outter.add_member("array_inner", ArrayType(inner, 4));
```


### DynamicData instantiation
```c++
DynamicData data(outter);
data["double"].value(6.7);
data["inner"]["m_int"].value(42);
data["inner"]["m_float"].value(35.8f);
data["string"].value<std::string>("Hi!");
data["string"].string("This is a string!");
data["seq_uint"].push(12);
data["seq_uint"].push(31);
data["seq_uint"][1].value(100);
data["seq_inner"].push(data["inner"]);
data["seq_inner"][0] = data["inner"];
data["array_uint"][1].value(123);
data["array_inner"][1] = data["om2"];
```


### QoS
- `NONE`
- `EQUALS`
- `IGNORE_TYPE_WIDTH`
- `IGNORE_SEQUENCE_BOUNDS`
- `IGNORE_ARRAY_BOUNDS`
- `IGNORE_STRING_BOUNDS`
- `IGNORE_MEMBER_NAMES`
- `IGNORE_OTHER_MEMBERS`
```c++
DynamicType t1;
//...
DynamicType t2;
//...

t1.is_compatible(t2)
```


### DynamicData operations
```c++
DynamicData d1(t1);
//...
DynamicData d2(t2);
//...

d1 == d2 //deep queality comparation
d1 != d2
d1 = d2 //same type assignation
```


#### Visitor
```c++
data.for_each([&](const DynamicData::ReadableNode& node)
{
    switch(node.data().type().kind())
    {
        case TypeKind::STRUCTURE_TYPE:
        case TypeKind::SEQUENCE_TYPE:
        case TypeKind::ARRAY_TYPE:
        case TypeKind::STRING_TYPE:
        case TypeKind::UINT_32_TYPE:
        case ...
        default:
    }

    node.data() // for a view of the data
    node.type() // related type
    node.deep() // nested deep
    node.parent() // parent node in the data tree
    node.access().index() // access from parent as index
    node.access().member() // access from parent as member
});
```


## DynamicData performance
```c++
StructType inner2("Inner2");
inner2.add_member("a", ArrayType(primitive_type<double>()));

StructType inner("Inner");
inner.add_member("b", ArrayType(inner2));
inner.add_member("c", primitive_type<float>());
inner.add_member("d", inner2);

StructType outter("Outter");
outter.add_member("e", ArrayType(inner));

DynamicData data(outter); // <- allocation here
```
###### Outter -> Array -> Inner -> Array -> Inner2 -> Array -> Double
Only one allocation!



## System Handle interface changes


### `configure` function
Previous
```c++
bool configure(
    const RequiredTypes& types,
    const YAML::Node& configuration) = 0;
```
Now
```c++
bool configure(
    const RequiredTypes& types,
    const YAML::Node& configuration,
    TypeRegistry& type_registry) = 0;

//std::map<std::string, xtypes::DynamicType::Ptr>
```


### `subscribe` function
Previous
```c++
bool subscribe(
    const std::string& topic_name,
    const std::string& message_type,
    SubscriptionCallback callback,
    const YAML::Node& configuration) = 0;
```
Now
```c++
bool subscribe(
    const std::string& topic_name,
    const xtypes::DynamicType& message_type,
    SubscriptionCallback callback,
    const YAML::Node& configuration) = 0;
```


### `advertise` function
Previous
```c++
std::shared_ptr<TopicPublisher> advertise(
    const std::string& topic_name,
    const std::string& message_type,
    const YAML::Node& configuration) = 0;
```
Now
```c++
std::shared_ptr<TopicPublisher> advertise(
    const std::string& topic_name,
    const xtypes::DynamicType& message_type,
    const YAML::Node& configuration) = 0;
```


### `publish` function
Previous
```c++
bool publish(const Message& message) = 0;
```
Now
```c++
bool publish(const xtypes::DynamicData& message) = 0;
```



## YAML changes


### System types configuration
Previously
```yaml
systems:
    ros2: { type: ros2 }
    mock: { type: mock }
```
Now (optional parameter)
```yaml
systems:
    ros2: { type: ros2 }
    mock: { type: mock, types-from: xtypes }
```
### Topic remap


Previously
```yaml
topics:
    pos2: { type: coord2d, route: s1_to_s2,
        remap: { s2: pos3 } } }
```
Now
```yaml
topics:
    pos2: { type: coord2d, route: s1_to_s2,
        remap: { s2: { topic: pos3, type: coord3d } } }
```



## Future work
- Qos (affect the behaviour of `is_compatible`)
    - Ignore members.
    - Ignore member names.
    - Type widening.
- Good documentation with examples.
- Unions and enums.
- Update DDS and FIWARE System Handles to use with xtypes (DDS will works natively).
