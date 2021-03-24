#include <is/json/conversion.hpp>

#include <iostream>
#include <stack>

using namespace eprosima::is::json;
using xtypes = eprosima::xtypes;

int main()
{
    xtypes::StructType inner("Inner");
    inner.add_member("int", xtypes::primitive_type<int>());
    inner.add_member("double", xtypes::primitive_type<double>());

    xtypes::StructType outer("Outer");
    outer.add_member("int", xtypes::primitive_type<int>());
    outer.add_member("string", xtypes::StringType());
    outer.add_member("array", xtypes::ArrayType(xtypes::primitive_type<int>(), 5));
    outer.add_member("sequence", xtypes::SequenceType(xtypes::primitive_type<double>()));
    outer.add_member("inner", inner);
    outer.add_member("inners", xtypes::SequenceType(inner));

    xtypes::DynamicData internal(inner);
    internal["int"] = 7;
    internal["double"] = 5.9;

    xtypes::DynamicData xtypes_message(outer);
    xtypes_message["int"] = 42;
    xtypes_message["string"] = "Hello json :D";
    for (size_t i = 0; i < xtypes_message["array"].size(); i++)
    {
        xtypes_message["array"][i] = 10 + int(i);
    }
    xtypes_message["sequence"].push(50.3);
    xtypes_message["sequence"].push(100.8);
    xtypes_message["inner"]["int"].value(1042);
    xtypes_message["inner"]["double"].value(10.42);

    xtypes_message["inners"].push(internal);
    xtypes_message["inners"].push(internal);


    Json json_message = {
        {"int", 42},
        {"array", {10, 11, 12, 13, 14}
        },
        {"string", "Hello json :D" },
        {"sequence", { 50.3, 100.8 }
        },
        {"inner",
         {
             {"int", 1042},
             {"double", 10.42}
         }, },
        {"inners",
         {
             {
                 {"int", 7},
                 {"double", 5.9}
             },
             {
                 {"int", 7},
                 {"double", 5.9}
             },
         }, },
    };

    std::cout << "ORIGINALS" << std::endl;
    std::cout << xtypes_message.to_string() << std::endl;
    std::cout << json_message.dump(4) << std::endl;

    xtypes::DynamicData from_json = convert(outer, json_message);
    Json from_xtypes = convert(xtypes_message);

    std::cout << "CONVERTED" << std::endl;
    std::cout << from_json.to_string() << std::endl;
    std::cout << from_xtypes.dump(4) << std::endl;

    std::stack<std::reference_wrapper<Json> > stack;
    stack.push(json_message["int"]);
    return 0;
}
