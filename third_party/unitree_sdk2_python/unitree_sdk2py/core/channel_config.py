import getpass
import os
import tempfile

# 本仓库改动：原版写死 /tmp/cdds.LOG。机器上别的用户（如 root 的服务）先建了它时，
# 当前用户写不进去，DDS 初始化直接失败。改成按用户区分的文件。
_CDDS_LOG = os.path.join(tempfile.gettempdir(), f"cdds-{getpass.getuser()}.LOG")

ChannelConfigHasInterface = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
                </Interfaces>
            </General>
            <Tracing>
                <Verbosity>config</Verbosity>
            <OutputFile>''' + _CDDS_LOG + '''</OutputFile>
        </Tracing>
        </Domain>
    </CycloneDDS>'''

ChannelConfigAutoDetermine = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\" />
                </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>'''
