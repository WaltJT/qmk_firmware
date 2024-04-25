/*
Copyright 2019 @foostan
Copyright 2020 Drashna Jaelre <@drashna>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include QMK_KEYBOARD_H
#include "quantum.h"
#include <stdio.h> //Inclusión obligatoria para poder escribir texto en la pantalla

enum layers {
  _QWERTY,
  _SIMBOLS,
  _NAVIGATE,
  _ADJUST,
  _DVORAK,
  _EMOJIS,
  _EMOJIS2
};

enum custom_keycodes {
  QWERTY = SAFE_RANGE,
  SIMBOLS,
  NAVIGATE,
  ADJUST,
  DVORAK,
  EMOJIS,
  EMOJIS2
};
         
// Tap Dance declarations
enum {
    TD_ESC_CAPS,
    TD_RALT_LALT,
    TD_HOME_END,
    TD_LCTL_AT,
};

enum unicode_names {
  BKSLH, // Backslash '( \ )'
  CIRC,  // Circa ^

  GRIN,  // grinning face 😊
  TJOY,  // tears of joy 😂
  SMILE, // grining face with smiling eyes 😁
  HEART, // heart ❤
  EYERT, // smiling face with heart shaped eyes 😍
  CRY,   // crying face 😭
  SMEYE, // smiling face with smiling eyes 😊
  UNAMU, // unamused 😒
  KISS,  // kiss 😘
  HART2, // two hearts 💕
  WEARY, // weary 😩
  OKHND, // ok hand sign 👌
  PENSV, // pensive 😔
  SMIRK, // smirk 😏
  RECYC, // recycle ♻
  WINK,  // wink 😉
  THMUP, // thumb up 👍
  THMDN, // thumb down 👎
  PRAY,  // pray 🙏
  PHEW,  // relieved 😌
  MUSIC, // musical notes
  FLUSH, // flushed 😳
  CELEB, // celebration 🙌
  CRY2,  // crying face 😢
  COOL,  // smile with sunglasses 😎
  NOEVS, // see no evil 🙈
  NOEVH, // hear no evil 🙉
  NOEVK, // speak no evil 🙊
  POO,   // pile of poo 💩
  EYES,  // eyes 👀
  VIC,   // victory hand ✌️
  BHART, // broken heart 💔
  SLEEP, // sleeping face 😴
  SMIL2, // smiling face with open mouth & sweat 😅
  HUNRD, // 100 💯
  CONFU, // confused 😕
  TONGU, // face with tongue & winking eye 😜
  DISAP, // disappointed 😞
  YUMMY, // face savoring delicious food 😋
  CLAP,  // hand clapping 👏
  FEAR,  // face screaming in fear 😱
  HORNS, // smiling face with horns 😈
  HALO,  // smiling face with halo 😇
  BYE,   // waving hand 👋
  SUN,   // sun ☀️
  MOON,  // moon 🌙
  SKULL, // skull 💀
  ROLF,  // Risa a carcajadas 🤣
  ZIPP,  // Zipper-mouthface 🤐
  RAT,   // Rat 🐀
  COW,   //Cow 🐄
  ELEPH, //Elephant 🐘
  DOG,   //Dog 🐕
  HRS,   //Horse 🐎
  BEER,  //Beer 🍺
  DRK,   //Clinking glasses (Drinks) 🥂
  BTL,   //bottle with popping cork 🍾
  JYT,   //Joystick 🎮
  PLC,   //Police officer 👮
  NOTE,  //Notebook 💻
  MNY,   //Heavy dollar sign (Money) 💲
  SHW,   //Shower 🛁
  CFC,   //Confounded face 😖
  PNC,   //Punch 👊
  MLW,   //Man lifting weights 🏋
  PWP,   //Paw prints 🐾

};

const uint32_t PROGMEM unicode_map[] = {
  [BKSLH] = 0x005C,
  [CIRC] = 0x005E,
    
  [GRIN] = 0x1F600,
  [TJOY] = 0x1F602,
  [SMILE] = 0x1F601,
  [HEART] = 0x2764,
  [EYERT] = 0x1f60d,
  [CRY] = 0x1f62d,
  [SMEYE] = 0x1F60A,
  [UNAMU] = 0x1F612,
  [KISS] = 0x1F618,
  [HART2] = 0x1F495,
  [WEARY] = 0x1F629,
  [OKHND] = 0x1F44C,
  [PENSV] = 0x1F614,
  [SMIRK] = 0x1F60F,
  [RECYC] = 0x267B,
  [WINK] = 0x1F609,
  [THMUP] = 0x1F44D,
  [THMDN] = 0x1F44E,
  [PRAY] = 0x1F64F,
  [PHEW] = 0x1F60C,
  [MUSIC] = 0x1F3B6,
  [FLUSH] = 0x1F633,
  [CELEB] = 0x1F64C,
  [CRY2] = 0x1F622,
  [COOL] = 0x1F60E,
  [NOEVS] = 0x1F648,
  [NOEVH] = 0x1F649,
  [NOEVK] = 0x1F64A,
  [POO] = 0x1F4A9,
  [EYES] = 0x1F440,
  [VIC] = 0x270C,
  [BHART] = 0x1F494,
  [SLEEP] = 0x1F634,
  [SMIL2] = 0x1F605,
  [HUNRD] = 0x1F4AF,
  [CONFU] = 0x1F615,
  [TONGU] = 0x1F61C,
  [DISAP] = 0x1F61E,
  [YUMMY] = 0x1F60B,
  [CLAP] = 0x1F44F,
  [FEAR] = 0x1F631,
  [HORNS] = 0x1F608,
  [HALO] = 0x1F607,
  [BYE] = 0x1F44B,
  [SUN] = 0x2600,
  [MOON] = 0x1F314,
  [SKULL] = 0x1F480,
  [ROLF] = 0x1F923,
  [ZIPP] = 0x1F910,
  [RAT] = 0x1F400,
  [COW] = 0X1F404,
  [ELEPH] = 0X1F418,
  [DOG] = 0X1F415,
  [HRS] = 0X1F40E,
  [BEER] = 0X1F37A,
  [DRK] = 0X1F942,
  [BTL] = 0X1F37E,
  [JYT] = 0X1F3AE,
  [PLC] = 0X1F46E,
  [NOTE] = 0X1F4BB,
  [MNY] = 0X1F4B2,
  [SHW] = 0X1F6C1,
  [CFC] = 0X1F616,
  [PNC] = 0X1F44A,
  [MLW] = 0X1F3CB,
  [PWP] = 0X1F43E,

};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_QWERTY] = LAYOUT(
  //,-----------------------------------------------------.                    ,-----------------------------------------------------.
TD(TD_ESC_CAPS),  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,                         KC_Y,    KC_U,    KC_I,    KC_O,    KC_P, KC_BSPC,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
LSFT_T(KC_LBRC),  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,                         KC_H,    KC_J,    KC_K,    KC_L, KC_SCLN,  KC_ENT,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
TD(TD_LCTL_AT),   KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,                         KC_N,    KC_M, KC_COMM,  KC_DOT, KC_SLSH,  KC_TAB, 
  //|--------+--------+--------+--------+--------+--------+--------|  |--------+--------+--------+--------+--------+--------+--------|
                                    KC_LGUI , TT(_SIMBOLS),  KC_SPC,     KC_SPC,TT(_NAVIGATE), TD(TD_RALT_LALT)
                                      //`--------------------------'  `--------------------------'

  ),

   [_SIMBOLS] = LAYOUT( 
  //,-----------------------------------------------------.                    ,-----------------------------------------------------.
       KC_ESC, KC_MINS,   KC_AT, KC_HASH,  KC_DLR, KC_NUBS,                      KC_PERC, KC_CIRC, KC_ASTR, KC_LPRN, KC_RPRN, KC_BSPC,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------| 
      KC_LSFT, KC_PGUP,   KC_UP, KC_PGDN, KC_RCBR, KC_AMPR,                       KC_EQL, KC_PLUS, KC_QUOT, KC_BSLS, X(CIRC),  KC_ENT,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
      KC_LCTL, KC_LEFT, KC_DOWN, KC_RIGHT, KC_GRV,X(BKSLH),                      KC_UNDS, KC_EXLM,  KC_DQT, KC_PIPE,RALT(KC_Q),KC_TAB, 
  //|--------+--------+--------+--------+--------+--------+--------|  |--------+--------+--------+--------+--------+--------+--------|
                                          KC_LGUI, _______,  KC_SPC,     KC_SPC,TT(_ADJUST), TD(TD_RALT_LALT)
                                      //`--------------------------'  `--------------------------'
  ),
  
    [_NAVIGATE] = LAYOUT(
  //,-----------------------------------------------------.                    ,-----------------------------------------------------.
       KC_ESC, KC_PGUP,   KC_UP, KC_PGDN, KC_MENU, KC_PSCR,                      KC_PSLS,    KC_1,    KC_2,    KC_3, KC_PPLS, KC_BSPC,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------| 
      KC_LSFT, KC_LEFT, KC_DOWN,KC_RIGHT, KC_AGIN,TD(TD_HOME_END),               KC_PAST,    KC_4,    KC_5,    KC_6, KC_PMNS,  KC_ENT,  
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
TD(TD_RALT_LALT), KC_CALC, KC_PEQL, KC_MUTE, KC_VOLD, KC_VOLU,                   KC_PCMM,    KC_7,    KC_8,    KC_9,    KC_0, KC_LCTL,
  //|--------+--------+--------+--------+--------+--------+--------|  |--------+--------+--------+--------+--------+--------+--------|
                                      KC_LGUI, TT(_DVORAK),  KC_SPC,     KC_SPC, _______, KC_DEL
                                      //`--------------------------'  `--------------------------'
  ),

    [_ADJUST] = LAYOUT(
  //,-----------------------------------------------------.                    ,-----------------------------------------------------.
      RGB_M_P, RGB_M_B, RGB_M_R,RGB_M_SW,RGB_M_SN, RGB_M_K,                      RGB_M_P, RGB_M_B, RGB_M_R,RGB_M_SW,RGB_M_SN, RGB_M_K,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
      RGB_MOD, RGB_HUI, RGB_SAI, RGB_VAI, RGB_SPI, RGB_M_X,                      RGB_MOD, RGB_HUI, RGB_SAI, RGB_VAI, RGB_SPI,  KC_ENT,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
     RGB_RMOD, RGB_HUD, RGB_SAD, RGB_VAD, RGB_SPD, RGB_M_G,                     RGB_RMOD, RGB_HUD, RGB_SAD, RGB_VAD, RGB_SPD, RGB_TOG,
  //|--------+--------+--------+--------+--------+--------+--------|  |--------+--------+--------+--------+--------+--------+--------|
                                          KC_LGUI, _______,  KC_SPC,     KC_SPC, _______, TT(_EMOJIS)
                                      //`--------------------------'  `--------------------------'
  ),

    [_DVORAK] = LAYOUT(
  //,-----------------------------------------------------.                    ,-----------------------------------------------------.
TD(TD_ESC_CAPS),KC_MINS,KC_COMM,  KC_DOT,    KC_P,    KC_Y,                         KC_F,    KC_G,    KC_C,    KC_R,    KC_L, KC_BSPC,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
LSFT_T(KC_LBRC),  KC_A,    KC_O,    KC_E,    KC_U,    KC_I,                         KC_D,    KC_H,    KC_T,    KC_N,    KC_S,  KC_ENT,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
TD(TD_LCTL_AT),KC_LABK,    KC_Q,    KC_J,    KC_K,    KC_X,                         KC_B,    KC_M,    KC_W,    KC_V,    KC_Z,  KC_TAB,
  //|--------+--------+--------+--------+--------+--------+--------|  |--------+--------+--------+--------+--------+--------+--------|
                                          KC_LGUI, _______,  KC_SPC,     KC_SPC, _______, TD(TD_RALT_LALT)
                                      //`--------------------------'  `--------------------------'

  ),

    [_EMOJIS] = LAYOUT(
  //,-----------------------------------------------------.                    ,-----------------------------------------------------.
       KC_ESC, X(GRIN),X(SMILE),X(EYERT),X(HEART),  X(CRY),                     X(SMILE),X(UNAMU), X(KISS),X(TJOY), X(WEARY), KC_BSPC,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
     X(RECYC),  X(CFC), X(WINK), X(COOL),X(OKHND),X(THMUP),                       X(SUN), X(PHEW),X(FLUSH),X(TONGU),X(SLEEP),  KC_ENT,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
     X(CONFU), X(PRAY), X(FEAR), X(ROLF),  X(POO),X(PENSV),                     X(SMEYE),X(SMIRK),X(BHART),X(DISAP),X(YUMMY),  KC_TAB,
  //|--------+--------+--------+--------+--------+--------+--------|  |--------+--------+--------+--------+--------+--------+--------|
                                      X(PNC), TT(_EMOJIS2),  KC_SPC,     KC_SPC, _______, _______
                                      //`--------------------------'  `--------------------------'
  ),

    [_EMOJIS2] = LAYOUT(
  //,-----------------------------------------------------.                    ,-----------------------------------------------------.
       X(VIC),X(HORNS), X(HALO), X(ZIPP),X(HART2), X(CRY2),                     X(SMIL2),X(MUSIC),X(NOEVS),X(NOEVH),X(NOEVK), KC_BSPC,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
       X(PWP),  X(COW),  X(DOG), X(BEER),X(HUNRD),X(THMDN),                      X(MOON),X(SKULL),  X(DRK), X(NOTE),  X(BTL),  KC_ENT,
  //|--------+--------+--------+--------+--------+--------|                    |--------+--------+--------+--------+--------+--------|
       X(RAT),X(ELEPH),  X(HRS),  X(PLC),X(CELEB), X(CLAP),                      X(EYES),  X(BYE),  X(MNY),  X(SHW),  X(JYT),  KC_TAB,
  //|--------+--------+--------+--------+--------+--------+--------|  |--------+--------+--------+--------+--------+--------+--------|
                                           X(MLW), _______,  KC_SPC,     KC_SPC, _______, _______
                                      //`--------------------------'  `--------------------------'
  ),

};

// Tap Dance definitions
tap_dance_action_t tap_dance_actions[] = {

    // Tap once for Escape, twice for Caps Lock
    [TD_ESC_CAPS] = ACTION_TAP_DANCE_DOUBLE(KC_ESC, KC_CAPS),
    [TD_RALT_LALT] = ACTION_TAP_DANCE_DOUBLE(KC_RALT, KC_LALT),
    [TD_HOME_END] = ACTION_TAP_DANCE_DOUBLE(KC_HOME, KC_END), 
    [TD_LCTL_AT] = ACTION_TAP_DANCE_DOUBLE(KC_LCTL, RALT(KC_Q)),

};

//CRKBD START

#ifdef OLED_ENABLE

oled_rotation_t oled_init_kb(oled_rotation_t rotation) {
    if (is_keyboard_master()) {
        return OLED_ROTATION_270; // flips the display 270 degrees if offhand
    } else {
        return OLED_ROTATION_180; // flips the display 180 degrees if offhand
    }
}


layer_state_t layer_state_set_user(layer_state_t state) { //Inclusión obligatoria si vamos a mostrar contenido diferente en cada capa
  if (is_keyboard_master()) {
  oled_clear();
  }
  return state;
}

//Tamaño del OLED 32 X 128 en vertical

//START ANIMATION

#define FRAME_DURATION 80 // Duración de cada frame en milisegundos

// variables
    uint32_t timer = 0;
    uint8_t current_frame = 0;

// Frames de la animación
    static void render_animationPika(void) {
    // 'frame-1', 32x23px
    static const char pikaframe_1 [] PROGMEM = {
    0x00, 0x00, 0x0e, 0x0e, 0x0e, 0x0e, 0x1c, 0xfc, 0xd8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xfc, 0xfc, 0xf0, 
    0xfc, 0xfc, 0xf0, 0xfe, 0xfe, 0xfc, 0xfc, 0xfc, 0x78, 0xf8, 0xb8, 0xe0, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x07, 
    0x07, 0x07, 0x0f, 0x3b, 0x1f, 0x0f, 0x07, 0x06, 0x06, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00,
    };
    // 'frame-2', 32x23px
    static const char pikaframe_2 [] PROGMEM = {
    0x38, 0x78, 0x38, 0x70, 0x70, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0xc0, 0xe0, 0xe0, 0xe0, 0xf8, 
    0xf8, 0xe0, 0xfc, 0xfd, 0xff, 0xff, 0xff, 0xfe, 0x9e, 0x9e, 0xfe, 0xec, 0x70, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 
    0x1f, 0x07, 0x07, 0x03, 0x03, 0x03, 0x0f, 0x0f, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    // 'frame-3', 32x23px
    static const char pikaframe_3 [] PROGMEM = {
    0x38, 0x38, 0x78, 0x78, 0x30, 0xf0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0xc0, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0xc0, 0xf0, 0xf0, 0xe0, 0xc8, 0xfc, 
    0xfc, 0xf0, 0xe0, 0xfc, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xcf, 0x73, 0x7e, 0x10, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0f, 0x0f, 0x0f, 0x0f, 0x07, 0x07, 
    0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00,
    };
    // 'frame-4', 32x23px
    static const char pikaframe_4 [] PROGMEM = {
    0x00, 0x02, 0x1e, 0x1e, 0x1e, 0x1c, 0x1c, 0x7c, 0x68, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xf8, 0xfc, 0xfe, 0xfe, 0xf8, 
    0xfe, 0xfe, 0xf0, 0xf8, 0xfe, 0xfe, 0xfc, 0xfc, 0x78, 0x78, 0xf8, 0xb0, 0xa0, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 
    0x01, 0x01, 0x03, 0x03, 0x0f, 0x1f, 0x7f, 0x37, 0x06, 0x06, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00
    };

	// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 1600)
	const char* pikaallArray[] = {
		pikaframe_1,
		pikaframe_2,
		pikaframe_3,
		pikaframe_4
    };

    //  Array de tamaños de los frames
        uint16_t frame_sizes[4] = {
            sizeof(pikaframe_1),
            sizeof(pikaframe_2),
            sizeof(pikaframe_3),
            sizeof(pikaframe_4)

    };


// Temporizador para que la animación funcione y cambie de frame
        if (timer_elapsed(timer) > FRAME_DURATION) {
            
            // Set timer to updated time
            timer = timer_read();
            
            // Increment frame
            current_frame = (current_frame + 1) % (sizeof(pikaallArray) / sizeof(pikaallArray[0]));

            // Dibujar la animación en la pantalla
            oled_write_raw_P(pikaallArray[current_frame], frame_sizes[current_frame]); 

    }
    
}

//END ANIMATION

static void oled_render_layer_state(void) {
    
    led_t led_state = host_keyboard_led_state();

    oled_write_P(PSTR("LAYER\n"), false);

    switch (get_highest_layer(layer_state)) {
        case 0: 
            oled_write_ln_P(PSTR(" QWE\n RTY\n"), false);
            oled_write_P(led_state.caps_lock ? PSTR("MAYUS\n CAP\n ") : PSTR("MAYUS\n \n "), false);
            oled_write_ln_P(PSTR(" \n"), false);
            break;
        case 1:
            oled_write_ln_P(PSTR(" SIMB OLS\n"), false);
            break;
        case 2:
            oled_write_ln_P(PSTR(" NAVI GATE\n"), false);
            break;
        case 3:
            oled_write_ln_P(PSTR(" ADJ\n UST\n"), false);
            break;
        case 4:
            oled_write_ln_P(PSTR(" DVO\n RAK\n"), false);
            oled_write_P(led_state.caps_lock ? PSTR("MAYUS\n CAP\n ")    : PSTR("MAYUS\n \n "), false);
            oled_write_ln_P(PSTR(" \n"), false);
            break;
        case 5:
            oled_write_ln_P(PSTR(" EMO\n JIS\n"), false);
            break;
        case 6:
            oled_write_ln_P(PSTR(" EMO\n JIS\n 2\n"), false);
            break;
        default:
            oled_write_ln_P(PSTR("Undef \n"), false);

    break;

    }
    render_animationPika();
};

/*
__attribute__((weak)) void oled_render_logo(void) {
    // clang-format off
    static const char PROGMEM crkbd_logo[] = {
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94,
        0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4,
        0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4,
        0};
    // clang-format on
    oled_write_P(crkbd_logo, false);
}

*/

void oled_render_logo(void) {
    static const char PROGMEM mb_logo[] = {
    // 'CORNE 2 WJT', 128x32px
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x1f, 0x0f, 0x0f, 0x0f, 0x1f, 0x1f, 0x3f, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0xc6, 
    0x00, 0x00, 0xff, 0xff, 0x7f, 0x3f, 0x3f, 0x3f, 0xff, 0x7f, 0x0f, 0x03, 0x01, 0x01, 0x00, 0xc0, 
    0xfc, 0x1f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xf0, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0x60, 0x30, 0x30, 0x30, 0x30, 0x20, 0x60, 
    0xc0, 0x80, 0x00, 0x00, 0xc0, 0xe0, 0x60, 0x30, 0x30, 0x30, 0x30, 0x20, 0x60, 0xe0, 0xc0, 0x00, 
    0x00, 0x00, 0xf0, 0xe0, 0x60, 0x30, 0x30, 0x30, 0x00, 0x00, 0xf0, 0xe0, 0x60, 0x30, 0x30, 0x30, 
    0x30, 0x60, 0xe0, 0xc0, 0x00, 0x00, 0x80, 0xc0, 0x60, 0x20, 0x30, 0x30, 0x30, 0x30, 0x60, 0xc0, 
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xe6, 0xe4, 0xa4, 0xbc, 0xbc, 0xb8, 0x19, 
    0x00, 0x00, 0xff, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
    0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xff, 0xff, 0xff, 0x7f, 0x1f, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x39, 0x60, 0x40, 0xc0, 0xc0, 0xc0, 0x40, 0x60, 
    0x30, 0x18, 0x00, 0x07, 0x3f, 0x38, 0x60, 0x40, 0xc0, 0xc0, 0xc0, 0x40, 0x60, 0x30, 0x3f, 0x0f, 
    0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x1f, 0x3f, 0x62, 0x42, 0xc2, 0xc2, 0xc2, 0x42, 0x62, 0x33, 
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x04, 0x04, 0x06, 0x06, 0x02, 0x03, 0x03, 
    0x00, 0x00, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xf8, 0xf8, 0xfc, 0xfe, 0xfc, 0xf8, 0xf8, 0xf8, 
    0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xf8, 0xf8, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xfc, 0xfe, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    oled_write_raw_P(mb_logo, sizeof(mb_logo));
    
}

bool oled_task_kb(void) {
    if (!oled_task_user()) {
        return false;
    }
    if (is_keyboard_master()) {
        oled_render_layer_state();
    } else {
        oled_render_logo();
    }
    return false;
}

#endif //END OLED

//CRKBD END

/*

EJEMPLO MT

#define KC_ESCC MT(MOD_LCTL, KC_ESC)
#define KC_ENTS MT(MOD_LSFT, KC_ENT)
#define KC_FN   MO(_FN) //_FN es una capa, esta definido de esa manera.

*/

/*

#ifdef RGB_MATRIX_ENABLE

void suspend_power_down_user(void) {
    rgb_matrix_set_suspend_state(true);
}

void suspend_wakeup_init_user(void) {
    rgb_matrix_set_suspend_state(false);
}

#endif

*/

// RGB_MODE_PLAIN	RGB_M_P 	Modo estático (sin animación)
// RGB_MODE_BREATHE	RGB_M_B	    Modo de animación de respiración
// RGB_MODE_RAINBOW	RGB_M_R	    Modo de animación arcoíris
// RGB_MODE_SWIRL	RGB_M_SW	Modo de animación de remolino
// RGB_MODE_SNAKE	RGB_M_SN	Modo de animación de serpiente
// RGB_MODE_KNIGHT	RGB_M_K	    Modo de animación "Knight Rider"
// RGB_MODE_XMAS	RGB_M_X	    Modo de animación navideña
// RGB_MODE_GRADIENT RGB_M_G	Modo de animación de degradado estático
// RGB_MODE_RGBTEST	RGB_M_T	    Modo de animación de prueba rojo, verde y azul   

/*

// QMK keycodes para la disposición en español latino
KC_NUBS,          // < MAYUS >
KC_GRV,           // | MAYUS °
KC_QUOT,          // { MAYUS [
KC_BSLS,          // } MAYUS ]
KC_NUHS,          // } MAYUS ]
KC_RBRC,          // + MAYUS *
KC_SCLN           // ñ MAYUS Ñ
KC_PSLS,          // /
KC_AMPR,          // /
KC_QUES,          // ¯
KC_LT,            // ;
KC_GT,            // :
KC_LCBR,          // ¨
KC_RCBR,          // *
KC_LABK,          // ;
KC_RABK,          // :
KC_TILD,          // °
KC_COLN,          // Ñ
KC_EXLM,          // !
KC_AT,            // "
KC_HASH,          // #
KC_DLR,           // $
KC_PERC,          // %
KC_CIRC,          // &
KC_ASTR,          // (
KC_LPRN,          // )
KC_RPRN,          // =
KC_EQL,           // ¿
KC_UNDS,          // ?
KC_PLUS,          // ¡
KC_PIPE,          // ]
KC_DQUO,          // [
KC_LBRC,          // ´
KC_MINS,          // '
KC_COMM,          // ,
KC_DOT,           // .
KC_SLSH,          // -

*/

