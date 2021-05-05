///! Helper Funcs for the Gcode parser

// Define buffer because the default only takes on command a line ...
pub enum Buffer {}

pub type GCode = gcode::GCode<ArgumentBuffer>;

type ArgumentBuffer = arrayvec::ArrayVec<[gcode::Word; 6]>;
type CommandBuffer = arrayvec::ArrayVec<[gcode::GCode<ArgumentBuffer>; 8]>;
type CommentBuffer<'input> = arrayvec::ArrayVec<[gcode::Comment<'input>; 0]>;

impl<'input> gcode::buffers::Buffers<'input> for Buffer {
    type Arguments = ArgumentBuffer;
    type Commands = CommandBuffer;
    type Comments = CommentBuffer<'input>;
}
