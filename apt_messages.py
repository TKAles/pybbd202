'''
    ThorLABS APT Protocol Message Registry
    Thomas Ales | Feb 2026
    Version 1
'''
import struct
from apt_constants import StatusBits as sb

class APTProtocol():
    ADDRESSES = { 'HOST_PC': 0x01, 'CONTROLLER': 0x11,
                  'X_AXIS': 0x21, 'Y_AXIS': 0x22
                }
    
    # N.B.: If the format is anything other than the following
    # two, it is considered a 'long' message. In this case
    # the format key refers only to the payload part of the
    # message. 
    # BB - Short Message, Only Source/Destination
    # BBBB - Short Message, Using Parameters 1 & 2
    
    # Dictionary entries MUST BE in the order they 
    # appear in the thorlabs documentation, if you don't
    # the unpacking logic goes all to 💩 and 'fun' things
    # happen.

    MSGS = {
        0x0002: { 
            'name': 'MGMSG_HW_DISCONNECT',
            'format': 'BB',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': None
        },
        0x0005: {
            'name': 'MGMSG_HW_REQ_INFO',
            'format': 'BB',
            'response': 0x0006,
            'fields': ['destination', 'source'],
            'data_fields': None
        },
        0x0006: {
            'name': 'MGMSG_HW_GET_INFO',
            'format': '<I8sH3B61xHHH',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': ['serial', 'model', 'type',
                            'fw_minor', 'fw_interim', 'fw_major',
                            'hw_ver', 'mod_state', 'num_channels']
        },
        0x0011: {
            'name': 'MGMSG_HW_START_UPDATEMSGS',
            'format': 'BB',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': None
        },
        0x0012: {
            'name': 'MGMSG_HW_STOP_UPDATEMSGS',
            'format': 'BB',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': None
        },
        0x0060: {
            'name': 'MGMSG_RACK_REQ_BAYUSED',
            'format': 'BBBB',
            'response': 0x0061,
            'fields': ['bay_id', 'destination', 'source'],
            'data_fields': None
        },
        0x0061: {
            'name': 'MGMSG_RACK_GET_BAYUSED',
            'format': 'BBBB',
            'response': None,
            'fields': ['bay_id', 'bay_state', 'destination', 'source'],
            'data_fields': None
        },
        0x0210: {
            'name': 'MGMSG_MOD_SET_CHANENABLESTATE',
            'format': 'BBBB',
            'response': None,
            'fields': ['chan_ident', 'enable_state', 'destination', 'source'],
            'data_fields': None
        },
        0x0211: {
            'name': 'MGMSG_MOD_REQ_CHANENABLESTATE',
            'format': 'BB',
            'response': 0x0212,
            'fields': ['destination', 'source'],
            'data_fields': None
        },
        0x0212: {
            'name': 'MGMSG_MOD_GET_CHANENABLESTATE',
            'format': 'BBBB',
            'response': None,
            'fields': ['chan_ident', 'enable_state', 'destination', 'source'],
            'data_fields': None
        },
        0x0413: {
            'name': 'MGMSG_MOT_SET_VELPARAMS',
            'format': '<Hlll',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': ['chan_ident', 'min_velocity', 'acceleration',
                            'max_velocity']
        },
        0x0414: {
            'name': 'MGMSG_MOT_REQ_VELPARAMS',
            'format': 'BBBB',
            'response': 0x0415,
            'fields': ['chan_ident', 'zero_this', 'destination', 'source'],
            'data_fields': None
        },
        0x0415: {
            'name': 'MGMSG_MOT_GET_VELPARAMS',
            'format': '<Hlll',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': ['chan_ident', 'min_velocity', 'acceleration',
                            'max_velocity']
        },
        0x0443: {
            'name': 'MGMSG_MOT_MOVE_HOME',
            'format': 'BBBB',
            'response': 0x0444,
            'fields': ['chan_ident', 'destination', 'source'],
            'data_fields': None
        },
        0x0444: {
            'name': 'MGMSG_MOT_MOVE_HOMED',
            'format': 'BBBB',
            'response': None,
            'fields': ['chan_ident', 'destination', 'source'],
            'data_fields': None
        },
        0x0448: {
            'name': 'MGMSG_MOT_MOVE_RELATIVE',
            'format': '<Hl',
            'response': 0x0464,
            'fields': ['destination', 'source'],
            'data_fields': ['chan_ident', 'relative_distance']
        },
        0x0453: {
            'name': 'MGMSG_MOT_MOVE_ABSOLUTE',
            'format': '<Hl',
            'response': 0x0464,
            'fields': ['destination', 'source'],
            'data_fields': ['chan_ident', 'absolute_distance']
        },
        0x0464: {
            'name': 'MGMSG_MOT_MOVE_COMPLETED',
            'format': '<Hl2HI',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': ['chan_ident', 'position', 'velocity',
                            'motor_current', 'status_bits']
        },
        0x0490: {
            'name': 'MGMSG_MOT_REQ_USTATUSUPDATE',
            'format': 'BB',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': None
        },
        0x0491: {
            'name': 'MGMSG_MOT_GET_USTATUSUPDATE',
            'format': '<Hl2HI',
            'response': 0x0492,
            'fields': ['destination', 'source'],
            'data_fields': ['chan_ident', 'position',
                            'velocity', 'motor_current', 'status_bits']
        },
        0x0492: {
            'name': 'MGMSG_MOT_ACK_USTATUSUPDATE',
            'format': 'BB',
            'response': None,
            'fields': ['destination', 'source'],
            'data_fields': None
        },
        0x0500: {
            'name': 'MGMSG_MOT_SET_TRIGGER',
            'format': 'BBBB',
            'response': None,
            'fields': ['chan_ident', 'mode', 'destination', 'source'],
            'data_fields': None
        },
        0x0501: {
            'name': 'MGMSG_MOT_REQ_TRIGGER',
            'format': 'BBBB',
            'response': 0x0502,
            'fields': ['chan_ident', 'mode', 'destination', 'source'],
            'data_fields': None
        },
        0x0502: {
            'name': 'MGMSG_MOT_GET_TRIGGER',
            'format': 'BBBB',
            'response': None,
            'fields': ['chan_ident', 'mode', 'destination', 'source'],
            'data_fields': None
        }

    }

    def __init__(self):
        pass

    @classmethod
    def build_message(cls, msg_id, **kwargs):
        if msg_id not in APTProtocol.MSGS:
            raise ValueError(f'Unknown Message ID: {hex(msg_id)}')

        msg_spec = APTProtocol.MSGS[msg_id]
        
        for param in msg_spec['fields']:
            if param not in kwargs:
                raise ValueError(f"Missing required parameter '{param}' for {msg_spec['name']}.")

        msg_format = msg_spec.get('format')

        if msg_format == 'BB':   
            # this is a simple source/destination command
            dest = kwargs['destination']
            src = kwargs['source']
            message = struct.pack('<HBBBB', msg_id, 
                                  0x00, 0x00,
                                  dest, src)

        elif msg_format == 'BBBB':
            # this is a source/destination plus parameters
            # fields order: [param1, param2, destination, source] or
            #               [param1, destination, source] (param2 = 0)
            fields = msg_spec['fields']
            dest = kwargs['destination']
            src = kwargs['source']
            # Get param fields (everything except destination/source)
            param_fields = [f for f in fields if f not in ('destination', 'source')]
            p1 = kwargs.get(param_fields[0], 0x00) if len(param_fields) > 0 else 0x00
            p2 = kwargs.get(param_fields[1], 0x00) if len(param_fields) > 1 else 0x00
            message = struct.pack('<HBBBB', msg_id,
                                  p1, p2, dest, src)

        else:
            data_fields = msg_spec.get('data_fields')
            # verify data fields are available
            if data_fields is None:
                raise ValueError("I need data for a long message!")

            # Extract the message specific values and
            # calculate payload size
            df_values = [kwargs[field] for field in data_fields]
            data_payload = struct.pack(msg_format, *df_values)
            payload_len = len(data_payload)
            header = struct.pack('<HHBB', msg_id, payload_len,
                                 kwargs['destination'] | 0x80,
                                 kwargs['source'])
            # put message together
            message = header + data_payload
        return message

    @classmethod
    def unpack_message(cls, bdata):
        # Get the message ID
        msg_id = struct.unpack("<H", bdata[0:2])[0]

        # See if the message ID has been implemented
        # in the registry
        if msg_id not in APTProtocol.MSGS:
            raise ValueError(f"This op-code {msg_id} isn't in the registry!")
        
        msg_spec = APTProtocol.MSGS[msg_id]
        fmt_string = msg_spec.get('format')
        # Check if it's a long message, or just a header
        if bdata[4] & 0x80:
            # this is a long message.
            payload_size = struct.unpack("<H", bdata[2:4])[0]
            src = bdata[5]
            dest = bdata[4] & 0x7F
            payload = bdata[6:6+payload_size]
            data_fields = msg_spec.get('data_fields')
            if data_fields is None:
                raise ValueError(f"No data fields have been defined for {msg_spec['name']}!")
            
            unpacked_payload = struct.unpack(fmt_string, payload)
            data = {field: value for field, value in zip(data_fields,
                                                         unpacked_payload)}
            data['destination'] = dest
            data['source'] = src

            return msg_id, data

        else:
            # it is a header only message
            # determine type
            dest = bdata[4]
            src = bdata[5]
            if fmt_string == 'BB':
                # simple source/dest command
                data = {'destination': dest, 'source': src}
                return msg_id, data

            elif fmt_string == 'BBBB':
                # simple command with parameters
                # Use field names from spec
                fields = msg_spec['fields']
                param_fields = [f for f in fields if f not in ('destination', 'source')]
                data = {'destination': dest, 'source': src}
                if len(param_fields) > 0:
                    data[param_fields[0]] = bdata[2]
                if len(param_fields) > 1:
                    data[param_fields[1]] = bdata[3]
                return msg_id, data
            else:
                raise ValueError(f"Message format {fmt_string} isn't defined/valid!")
    
    @classmethod
    def get_name(cls, msg_id):
        return cls.MSGS.get(msg_id, {}).get('name', f'UNKNOWN_{hex(msg_id)}')

