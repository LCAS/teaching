unset RMW_IMPLEMENTATION
unset ROS_LOCALHOST_ONLY
#export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/teaching/uol_tidybot/param/super_client_configuration_file.xml
#export ROS_DISCOVERY_SERVER=localhost:11888

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

read -r -d '' CYCLONEDDS_URI << EOM
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="0">
        <General>
            <AllowMulticast>false</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        <FragmentSize>4000B</FragmentSize>
        <Transport>udp</Transport>
        </General>
    <Discovery>
        <Peers>
            <Peer address="localhost"/>
            <Peer address="`hostname`"/>
            <!--<Peer address="[IPV6-address]"/>-->
        </Peers>
        <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
        <Tracing>
            <Verbosity>info</Verbosity>
            <OutputFile>stdout</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
EOM

export CYCLONEDDS_URI

# export CYCLONEDDS_URI='
# <?xml version="1.0" encoding="UTF-8" ?>
# <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
#     <Domain id="any">
#         <General>
#             <AllowMulticast>false</AllowMulticast>
#             <MaxMessageSize>65500B</MaxMessageSize>
#         <FragmentSize>4000B</FragmentSize>
#         <Transport>udp</Transport>
#         </General>
#     <Discovery>
#         <Peers>
#             <Peer address="localhost"/>
#             <Peer address="`hostname`"/>
#             <!--<Peer address="[IPV6-address]"/>-->
#         </Peers>
#         <ParticipantIndex>auto</ParticipantIndex>
#     </Discovery>
#         <Internal>
#             <Watermarks>
#                 <WhcHigh>500kB</WhcHigh>
#             </Watermarks>
#         </Internal>
#         <Tracing>
#             <Verbosity>info</Verbosity>
#             <OutputFile>stdout</OutputFile>
#         </Tracing>
#     </Domain>
# </CycloneDDS>
# '