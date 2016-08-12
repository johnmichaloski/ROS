# Crcl command contains information for canonical robot command language exchange.
# CRCL is expressed in XML.
# At a primite level, a CRCL message is a framed xml string.
# So, the ROS message contains a xml string and the originating ip from where the CRCL message came.

string xml
string ip
uint16 port
