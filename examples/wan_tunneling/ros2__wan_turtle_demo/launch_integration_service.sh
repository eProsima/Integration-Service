#!/bin/sh

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <wan_addr> <port> <yaml_file>"
    exit 2
fi

sed -r -i 's+<wan_addr>.*</wan_addr>+<wan_addr>'"$1"'</wan_addr>+g' wan_config.xml
sed -r -i 's+<address>.*</address>+<address>'"$1"'</address>+g' wan_config.xml
sed -r -i 's+<port>.*</port>+<port>'"$2"'</port>+g' wan_config.xml
sed -r -i 's+<physical_port>.*</physical_port>+<physical_port>'"$2"'</physical_port>+g' wan_config.xml

integration-service $3
