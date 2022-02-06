/* GIMP RGB C-Source image dump (ws2812b.c) */

static const struct {
  unsigned int 	 width;
  unsigned int 	 height;
  unsigned int 	 bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */ 
  unsigned char	 pixel_data[16 * 16 * 3 + 1];
} ws2812b = {
  16, 16, 3,
  "\067\067\067\071\071\071:\071:\065\065\065\064\064\065\062\062\062\061\061\062\060\061\061\060\060"
  "\060\061\061\061\063\063\064\062\062\062\063\063\063\063\064\064\066\066\067\065\065\065\066\066"
  "\066\071\071\071\071\071\071\067\067\067\067\067\067\067\067\067\067\067\067\066\066\066\065\065"
  "\065\067\067\067\067\067\067\066\066\066\066\066\066\065\065\065\065\065\065\065\065\065\065\065"
  "\065\065\065\065\066\066\066\067\067\067\067\067\067\061\060\060+*))(&('&++)//.\063\063\063"
  "\066\066\066\070\070\070\067\067\067\067\067\067\060\060\060\065\065\065\067\067\067\065\065\065"
  "+*)\"!\037*)&\024\023\023\021\020\020\021\021\020\033\033\032)*(\060\060\060\067\067\067\070"
  "\070\070\067\067\067---\067\067\067\067\067\067**(\034\034\033\022\023\021*($\023\023\021\016"
  "\017\014\014\014\014\013\013\012\020\020\016))&\061\061\061\067\067\067\067\067\067\060\060\060"
  "\067\067\070\061\060\060\037\037\035\021\021\020\021\022\020\"\040\034!\037\033\020\020\013\017"
  "\017\007\011\011\011\026\026\025''&---\067\067\067\067\070\070\061\062\062\067\067\067)(&\024\025"
  "\023\014\020\017\012\014\012\016\016\012&\"\034$\"\034\"\037\033\"\037\033**'.-,,+*\063\063"
  "\063\067\067\070\062\062\064\067\067\067#\"!\022\023\021\012\017\017\000\000\000\000\000\000\035\031\022"
  "%\"\035!\034\030\035\034\030\037\036\034!\037\036(&&\060\060\060\070\067\070\060\061\062\066\066"
  "\065\"\"\037\024\025\022\013\020\017\000\000\000\000\000\000\033\026\016\036\032\022\000\000\000\013\012\007"
  "\011\012\011\013\013\012\"!\040\061\061\061\067\070\070.//\067\067\067$$\"\026\026\024\024\025"
  "\023\006\004\000\000\000\000$!\030\035\034\024\021\020\013\013\014\012\012\012\011\012\012\012%%$\064\064"
  "\065\067\067\067\062\062\065\067\067\070,+*'&$*(&*'\"*'#+)$\026\026\021\030\025\017\017\017"
  "\020\016\016\016\024\024\024---\067\067\067\067\067\067\064\064\065\067\067\067\065\064\064&%#"
  "\031\031\027\024\023\017\033\032\027)(#\025\026\023\023\024\022\024\024\024\027\027\027('&\063"
  "\064\063\067\067\067\067\067\067\062\062\062\067\067\067\067\067\067\061\060\060'&%\037\037\034"
  "\036\035\034*)&\035\036\034\035\035\034\"!\040)(&\060\060.\067\067\067\070\070\070\070\070\070"
  "\060\060\060\067\067\067\070\070\070\070\070\070\063\063\062++))'&*)'('&*)',+*\062\062\060"
  "\067\067\067\070\070\070\070\070\070\070\070\070\060\060\060\066\066\066\067\067\067\070\070\070"
  "\067\067\070\067\067\067\064\064\063\062\062\062\062\062\061\064\064\064\067\067\067\070\070\070"
  "\070\070\070\070\070\070\070\070\070\067\067\067-,-\064\063\064\067\067\067\067\067\067\067\066"
  "\067\067\066\067\065\065\065\066\065\065\065\066\065\065\065\065\063\064\064\065\065\065\066\065"
  "\065\067\067\067\066\066\066\065\065\065",
};

