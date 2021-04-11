#include "src/blinks-broadcast/handler.h"
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
static int8_t current_y_;

static void consume(const broadcast::Message* message,
                    byte absolute_local_face) {
  timer_.set(MAPPING_TIMEOUT_MS);

  if (!mapping::Initialized()) {
    position::Setup(orientation::RelativeLocalFace(absolute_local_face),
                    (int8_t)message->payload[0], (int8_t)message->payload[1]);

    mapping::Set(position::Local().x, position::Local().y,
                 MAPPING_SET_NEW_VALUE(1));
  }

  if (mapping::Get((int8_t)message->payload[0], (int8_t)message->payload[1]) !=
      MAPPING_POSITION_EMPTY) {
    return;
  }

  mapping::Set((int8_t)message->payload[0], (int8_t)message->payload[1],
               MAPPING_SET_NEW_VALUE(message->payload[2]));
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

static void maybe_propagate() {
  byte local_value = mapping::Get(position::Local().x, position::Local().y);

  if (MAPPING_IS_NEW_VALUE(local_value)) {
    byte real_value = MAPPING_UNSET_NEW_VALUE(local_value);

    broadcast::Message message;
    message.header.id = MAPPING_MESSAGE_PROPAGATE_COORDINATES;
    message.payload[0] = position::Local().x;
    message.payload[1] = position::Local().y;
    message.payload[2] = real_value;

    if (broadcast::manager::Send(&message)) {
      mapping::Set(position::Local().x, position::Local().y, real_value);
    }

    return;
  }

  mapping::AllPositions([](int8_t x, int8_t y, byte* value) -> bool {
    if (MAPPING_IS_NEW_VALUE(*value)) {
      byte real_value = MAPPING_UNSET_NEW_VALUE(*value);

      broadcast::Message message;
      message.header.id = MAPPING_MESSAGE_PROPAGATE_COORDINATES;
      message.payload[0] = x;
      message.payload[1] = y;
      message.payload[2] = real_value;

      if (broadcast::manager::Send(&message)) {
        *value = real_value;
      }

      return false;
    }

    return true;
  });
}

static void process() {
  broadcast::manager::Process();

  orientation_from_face_value();

  maybe_propagate();
}

void setup() {
  // Setup external message handler.
  broadcast::message::handler::Set(
      {MAPPING_MESSAGE_PROPAGATE_COORDINATES, consume});
}

void loop() {
  process();

  bool has_woken = hasWoken();
  bool button_single_clicked = buttonSingleClicked();

  if (button_single_clicked && !has_woken) {
    timer_.set(MAPPING_TIMEOUT_MS);

    // TODO(bga): Fix this. We need a way to pass the value for the local Blink
    // here.
    mapping::Set(position::Local().x, position::Local().y,
                 MAPPING_SET_NEW_VALUE(1));
  }

  if (!timer_.isExpired()) {
    setColor(GREEN);
    dumped_ = false;
  } else {
    setColor(BLUE);
    if (!dumped_) {
      mapping::AllPositions([](int8_t x, int8_t y, byte* value) -> bool {
        if (y != current_y_) {
          LOGFLN("");
          current_y_ = y;
        }

        LOG(*value);
        LOGF(" ");

        return true;
      });

      LOGFLN("");

      dumped_ = true;
    }
  }

  setColorOnFace(WHITE, orientation::AbsoluteLocalFace(0));
}
