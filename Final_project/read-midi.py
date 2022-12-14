"""
Python script to read midi file and parse events. 
args: midi file path
eg. python <script-path> <midi-file-path>

Script prompts user to choose track, and then parses sequence of midi events 
corresponding to the particular track selected. 
Events are written in the form of an array of C structs and written to 
the song.c file as metadata to be read when playing a song. 

Each event is represented by 3 fields - notes_press, notes_release, hold_time. 
notes_press: an integer representing midi note number to press down 
notes_release: an integer representing midi note number to release 
(previously pressed)
hold_time: time to wait before pressing/releasing specified note. 
If no key to press/release, default value of -1 is passed. 
"""

import os
import sys
from mido import MidiFile
from csnake import CodeWriter, Variable, FormattedLiteral

mid = MidiFile(sys.argv[1], clip=True)

for i in range(len(mid.tracks)):
    print(str(i) + "." + str(mid.tracks[i]))
track_num = int(input("Choose track: "))
print(mid.tracks[track_num])

song_data = []
all_notes = []

for msg in mid.tracks[track_num]:
    # print(msg)
    if msg.type == 'note_on':

        # if playing note, add event with note press
        if msg.velocity > 0:
            all_notes.append(msg.note - 12)
            song_data.append({
                'notes_press': msg.note - 12,
                'notes_release': -1,
                'hold_time': msg.time
            })

        # if velocity is 0, release note
        else:
            song_data.append({
                'notes_press': -1,
                'notes_release': msg.note - 12,
                'hold_time': msg.time
            })
    elif msg.type == 'note_off':
        # release note
        song_data.append({
            'notes_press': -1,
            'notes_release': msg.note - 12,
            'hold_time': msg.time
        })

var = Variable(
    "song_data1",
    primitive="note_t",
    value=song_data
)
length = Variable(
    "song_len1",
    primitive="long",
    value=len(song_data)
)
cw = CodeWriter()
cw.add_variable_initialization(var)
cw.add_variable_initialization(length)

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, 'song.c')
f = open(filename, "a")
f.write('#include "song.h"\n')
f.write(str(length) + "\n")
f.write(str(var))
f.close()

print("Note Range:")
print("min", min(all_notes))
print("max", max(all_notes))
