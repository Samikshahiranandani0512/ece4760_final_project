
typedef struct note_event {
    int notes_press;   // which note to play
    int notes_release; // which note to release
    int hold_time;     // number of ticks to hold for
} note_t;

extern note_t song_data1[];
extern long song_len1;

extern note_t song_data2[];
extern long song_len2;

extern note_t song_data3[];
extern long song_len3;

extern note_t song_data4[];
extern long song_len4;

extern note_t song_data5[];
extern long song_len5;
