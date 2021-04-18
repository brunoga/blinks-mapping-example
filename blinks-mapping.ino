#include "src/blinks-broadcast/manager.h"
#include "src/blinks-debug/debug.h"
#include "src/blinks-mapping/mapping.h"
#include "src/blinks-orientation/orientation.h"
#include "src/blinks-position/position.h"

#define MAPPING_MESSAGE_PROPAGATE_COORDINATES 1

#define MAPPING_TIMEOUT_MS 5000

#define MAPPING_SET_NEW_VALUE(v) (v | 0b10000000)
#define MAPPING_IS_NEW_VALUE(v) ((v & 0b10000000) != 0)
#define MAPPING_UNSET_NEW_VALUE(v) (v & 0b01111111)

static Timer timer_;
static bool was_alone_;
static bool dumped_;

bool external_message_handler(byte absolute_local_face,
                              const broadcast::Message* message) {
  if (message->header.id != MAPPING_MESSAGE_PROPAGATE_COORDINATES) return false;

  timer_.set(MAPPING_TIMEOUT_MS);

  if (!mapping::Initialized()) {
    position::Setup(orientation::RelativeLocalFace(absolute_local_face),
                    (int8_t)message->payload[0], (int8_t)message->payload[1]);

    mapping::Set(position::Local().x, position::Local().y,
                 MAPPING_SET_NEW_VALUE(1));
  }

  if (mapping::Get((int8_t)message->payload[0], (int8_t)message->payload[1]) ==
      MAPPING_POSITION_EMPTY) {
    mapping::Set((int8_t)message->payload[0], (int8_t)message->payload[1],
                 MAPPING_SET_NEW_VALUE(message->payload[2]));
  }

  return true;
}

static void orientation_from_face_value() {
  if (isAlone()) {
    was_alone_ = true;
  } else if (was_alone_) {
    was_alone_ = false;
    orientation::Setup();
  } else {
    FOREACH_FACE(face) {
      if (!didValueOnFaceChange(face)) continue;

      byte face_value = getLastValueReceivedOnFace(face);

      if (face_value == 0) continue;

      orientation::Setup(face_value - 1, face);

      break;
    }
  }

  FOREACH_FACE(face) {
    setValueSentOnFace(orientation::RelativeLocalFace(face) + 1, face);
  }
}

static void maybe_send_value(int8_t x, int8_t y, byte value) {
  if (MAPPING_IS_NEW_VALUE(value)) {
    byte real_value = MAPPING_UNSET_NEW_VALUE(value);

    broadcast::Message message;
    message.header.id = MAPPING_MESSAGE_PROPAGATE_COORDINATES;
    message.payload[0] = x;
    message.payload[1] = y;
    message.payload[2] = real_value;

    if (broadcast::manager::Send(&message)) {
      mapping::Set(x, y, real_value);
    }
  }
}

static void maybe_propagate() {
  if (!mapping::Initialized()) return;

  maybe_send_value(position::Local().x, position::Local().y,
                   mapping::Get(position::Local().x, position::Local().y));

  mapping::AllValidPositions(
      [](int8_t x, int8_t y, byte* value, void* context) -> bool {
        maybe_send_value(x, y, *value);

        return true;
      },
      nullptr);
}

static void process() {
  broadcast::manager::Process();

  orientation_from_face_value();

  maybe_propagate();
}

void setup() {}

void loop() {
  process();

  bool has_woken = hasWoken();
  bool button_single_clicked = buttonSingleClicked();

  if (button_single_clicked && !has_woken) {
    timer_.set(MAPPING_TIMEOUT_MS);

    mapping::Set(position::Local().x, position::Local().y,
                 MAPPING_SET_NEW_VALUE(1));
  }

  if (!timer_.isExpired()) {
    setColor(GREEN);
    dumped_ = false;
  } else {
    setColor(BLUE);
    if (!dumped_) {
      mapping::AllValidPositions(
          [](int8_t x, int8_t y, byte* value, void* context) -> bool {
            LOG(x);
            LOGF(",");
            LOG(y);
            LOGF(" -> ");
            LOGLN(*value);

            return true;
          },
          nullptr);

      dumped_ = true;
    }
  }

  setColorOnFace(WHITE, orientation::AbsoluteLocalFace(0));
}
