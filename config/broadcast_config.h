#ifndef BROADCAST_CONFIG_H_
#define BROADCAST_CONFIG_H_

namespace broadcast {

struct Message;

}  // namespace broadcast

bool external_message_handler(byte absolute_local_face,
                              const broadcast::Message* message);

#define BROADCAST_MESSAGE_PAYLOAD_BYTES 3
#define BROADCAST_DISABLE_REPLIES

#define BROADCAST_EXTERNAL_MESSAGE_HANDLER external_message_handler

#endif  // BROADCAST_CONFIG_H_