using System;
using System.Linq;
using unique_identifier_msgs.msg;

namespace ROS2
{
    public static class GuidExtensions
    {
        public static UUID ToUuidMsg(this Guid guid)
        {
            // Microsoft uses some mixed endian representation for Guid in
            // Windows and .NET, which the Guid.ToByteArray() does as well.
            // Most/All other implementations of UUID/GUIDs use an unique big
            // endian representation on the wire. See this StackOverflow answer
            // for more context: https://stackoverflow.com/a/9195681
            //
            // So there is a need to swap some bytes before converting them to
            // the UUID ROS message.
            //
            // Even on BigEndian machines .NET does the swapping of the first
            // three fields to little endian, so it is at least consistent on
            // all platforms, no need to check the endianness of the runtime.
            // https://github.com/dotnet/runtime/blob/1ba0394d71a4ea6bee7f6b28a22d666b7b56f913/src/libraries/System.Private.CoreLib/src/System/Guid.cs#L819
            var guidBytes = guid.ToByteArray();
            EndianSwap(guidBytes);

            var uuidMsg = new UUID();
            uuidMsg.Uuid = guidBytes;
            return uuidMsg;
        }

        public static byte[] ToUuidByteArray(this Guid guid)
        {
            // See comments in `ToUuidMsg` for info on byte swapping.

            var guidBytes = guid.ToByteArray();
            EndianSwap(guidBytes);

            return guidBytes;
        }

        public static Guid ToGuid(this UUID uuidMsg)
        {
            // See comments in `ToUuidMsg` for info on byte swapping.

            var uuidBytesCopy = uuidMsg.Uuid.ToArray();
            EndianSwap(uuidBytesCopy);

            var guid = new Guid(uuidBytesCopy);
            return guid;
        }

        private static void EndianSwap(byte[] guid)
        {
            // taken from https://github.com/StephenCleary/Guids/blob/a9bd91835d536636d13249bd1bfbb1cd76c43533/src/Nito.Guids/GuidUtility.cs#L47
            Swap(guid, 0, 3);
            Swap(guid, 1, 2);

            Swap(guid, 4, 5);

            Swap(guid, 6, 7);
        }

        private static void Swap(byte[] array, int index1, int index2)
        {
            var temp = array[index1];
            array[index1] = array[index2];
            array[index2] = temp;
        }
    }
}
