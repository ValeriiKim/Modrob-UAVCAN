// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://uavcan.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.5.1 (serialization was enabled)
// Source file:   /home/xbron/Projects/cpp/public_regulated_data_types-master/modrob/sensor_module/sensor_data.0.2.uavcan
// Generated at:  2022-05-11 13:27:33.506688 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     modrob.sensor_module.sensor_data
// Version:       0.2
//
// Platform
//     python_implementation:  CPython
//     python_version:  3.6.9
//     python_release_level:  final
//     python_build:  ('default', 'Mar 15 2022 13:55:28')
//     python_compiler:  GCC 8.4.0
//     python_revision:
//     python_xoptions:  {}
//     runtime_platform:  Linux-5.4.0-107-generic-x86_64-with-Ubuntu-18.04-bionic
//
// Language Options
//     target_endianness:  little
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  True
//     enable_override_variable_array_capacity:  False

#ifndef MODROB_SENSOR_MODULE_SENSOR_DATA_0_2_INCLUDED_
#define MODROB_SENSOR_MODULE_SENSOR_DATA_0_2_INCLUDED_

#include <modrob/sensor_module/obst_info_0_1.h>
#include <modrob/sensor_module/segment_params_0_1.h>
#include <nunavut/support/serialization.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 434322821,
              "/home/xbron/Projects/cpp/public_regulated_data_types-master/modrob/sensor_module/sensor_data.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/xbron/Projects/cpp/public_regulated_data_types-master/modrob/sensor_module/sensor_data.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 1,
              "/home/xbron/Projects/cpp/public_regulated_data_types-master/modrob/sensor_module/sensor_data.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 0,
              "/home/xbron/Projects/cpp/public_regulated_data_types-master/modrob/sensor_module/sensor_data.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define modrob_sensor_module_sensor_data_0_2_HAS_FIXED_PORT_ID_ false

#define modrob_sensor_module_sensor_data_0_2_FULL_NAME_             "modrob.sensor_module.sensor_data"
#define modrob_sensor_module_sensor_data_0_2_FULL_NAME_AND_VERSION_ "modrob.sensor_module.sensor_data.0.2"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define modrob_sensor_module_sensor_data_0_2_EXTENT_BYTES_                    2610UL
#define modrob_sensor_module_sensor_data_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_ 2610UL
static_assert(modrob_sensor_module_sensor_data_0_2_EXTENT_BYTES_ >= modrob_sensor_module_sensor_data_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// saturated uint8 MAX_OBST_NUM = 10
#define modrob_sensor_module_sensor_data_0_2_MAX_OBST_NUM (10U)
/// saturated uint16 MAX_SEG_NUM = 150
#define modrob_sensor_module_sensor_data_0_2_MAX_SEG_NUM (150U)

/// Array metadata for: saturated float32[2] cur_pos
#define modrob_sensor_module_sensor_data_0_2_cur_pos_ARRAY_CAPACITY_           2U
#define modrob_sensor_module_sensor_data_0_2_cur_pos_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: modrob.sensor_module.obst_info.0.1[<=10] obstacles
#define modrob_sensor_module_sensor_data_0_2_obstacles_ARRAY_CAPACITY_           10U
#define modrob_sensor_module_sensor_data_0_2_obstacles_ARRAY_IS_VARIABLE_LENGTH_ true
/// Array metadata for: modrob.sensor_module.segment_params.0.1[<=150] segments
#define modrob_sensor_module_sensor_data_0_2_segments_ARRAY_CAPACITY_           150U
#define modrob_sensor_module_sensor_data_0_2_segments_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// saturated float32[2] cur_pos
    float cur_pos[2];

    /// modrob.sensor_module.obst_info.0.1[<=10] obstacles
    struct  /// Array address equivalence guarantee: &elements[0] == &obstacles
    {
        modrob_sensor_module_obst_info_0_1 elements[modrob_sensor_module_sensor_data_0_2_obstacles_ARRAY_CAPACITY_];
        size_t count;
    } obstacles;

    /// modrob.sensor_module.segment_params.0.1[<=150] segments
    struct  /// Array address equivalence guarantee: &elements[0] == &segments
    {
        modrob_sensor_module_segment_params_0_1 elements[modrob_sensor_module_sensor_data_0_2_segments_ARRAY_CAPACITY_];
        size_t count;
    } segments;
} modrob_sensor_module_sensor_data_0_2;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see modrob_sensor_module_sensor_data_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t modrob_sensor_module_sensor_data_0_2_serialize_(
    const modrob_sensor_module_sensor_data_0_2* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    if ((8U * (size_t) capacity_bytes) < 20880UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated float32[2] cur_pos
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 64ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- assume the native representation is conformant.
        static_assert(NUNAVUT_PLATFORM_IEEE754_FLOAT, "Native IEEE754 binary32 required. TODO: relax constraint");
        // Optimization prospect: this item is aligned at the byte boundary, so it is possible to use memmove().
        nunavutCopyBits(&buffer[0], offset_bits, 2UL * 32UL, &obj->cur_pos[0], 0U);
        offset_bits += 2UL * 32UL;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad0_ > 0);
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += _pad0_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }

    {   // modrob.sensor_module.obst_info.0.1[<=10] obstacles
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 1608ULL) <= (capacity_bytes * 8U));
        if (obj->obstacles.count > 10)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->obstacles.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        for (size_t _index0_ = 0U; _index0_ < obj->obstacles.count; ++_index0_)
        {
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits + 160ULL) <= (capacity_bytes * 8U));
            size_t _size_bytes0_ = 20UL;  // Nested object (max) size, in bytes.
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes0_) <= capacity_bytes);
            int8_t _err1_ = modrob_sensor_module_obst_info_0_1_serialize_(
                &obj->obstacles.elements[_index0_], &buffer[offset_bits / 8U], &_size_bytes0_);
            if (_err1_ < 0)
            {
                return _err1_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            NUNAVUT_ASSERT((_size_bytes0_ * 8U) == 160ULL);
            offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
            NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
        }
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad1_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad1_ > 0);
        const int8_t _err2_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad1_);  // Optimize?
        if (_err2_ < 0)
        {
            return _err2_;
        }
        offset_bits += _pad1_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }

    {   // modrob.sensor_module.segment_params.0.1[<=150] segments
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 19208ULL) <= (capacity_bytes * 8U));
        if (obj->segments.count > 150)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->segments.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        for (size_t _index1_ = 0U; _index1_ < obj->segments.count; ++_index1_)
        {
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits + 128ULL) <= (capacity_bytes * 8U));
            size_t _size_bytes1_ = 16UL;  // Nested object (max) size, in bytes.
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes1_) <= capacity_bytes);
            int8_t _err3_ = modrob_sensor_module_segment_params_0_1_serialize_(
                &obj->segments.elements[_index1_], &buffer[offset_bits / 8U], &_size_bytes1_);
            if (_err3_ < 0)
            {
                return _err3_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            NUNAVUT_ASSERT((_size_bytes1_ * 8U) == 128ULL);
            offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested object.
            NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
        }
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad2_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad2_ > 0);
        const int8_t _err4_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad2_);  // Optimize?
        if (_err4_ < 0)
        {
            return _err4_;
        }
        offset_bits += _pad2_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.

    NUNAVUT_ASSERT(offset_bits >= 80ULL);
    NUNAVUT_ASSERT(offset_bits <= 20880ULL);

    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (offset_bits / 8U);

    return NUNAVUT_SUCCESS;
}

/// Deserialize an instance from the provided buffer.
/// The lifetime of the resulting object is independent of the original buffer.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original buffer where possible.
///
/// @param obj      The object to update from the provided serialized representation.
///
/// @param buffer   The source buffer containing the serialized representation. There are no alignment requirements.
///                 If the buffer is shorter or longer than expected, it will be implicitly zero-extended or truncated,
///                 respectively; see Specification for "implicit zero extension" and "implicit truncation" rules.
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the supplied serialized
///                                 representation, in bytes. Upon return this value will be updated with the
///                                 size of the consumed fragment of the serialized representation (in bytes),
///                                 which may be smaller due to the implicit truncation rule, but it is guaranteed
///                                 to never exceed the original buffer size even if the implicit zero extension rule
///                                 was activated. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t modrob_sensor_module_sensor_data_0_2_deserialize_(
    modrob_sensor_module_sensor_data_0_2* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated float32[2] cur_pos
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    static_assert(NUNAVUT_PLATFORM_IEEE754_FLOAT, "Native IEEE754 binary32 required. TODO: relax constraint");
    nunavutGetBits(&out_obj->cur_pos[0], &buffer[0], capacity_bytes, offset_bits, 2UL * 32U);
    offset_bits += 2UL * 32U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // modrob.sensor_module.obst_info.0.1[<=10] obstacles
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->obstacles.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->obstacles.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->obstacles.count > 10U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    for (size_t _index2_ = 0U; _index2_ < out_obj->obstacles.count; ++_index2_)
    {
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        {
            size_t _size_bytes2_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            const int8_t _err5_ = modrob_sensor_module_obst_info_0_1_deserialize_(
                &out_obj->obstacles.elements[_index2_], &buffer[offset_bits / 8U], &_size_bytes2_);
            if (_err5_ < 0)
            {
                return _err5_;
            }
            offset_bits += _size_bytes2_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // modrob.sensor_module.segment_params.0.1[<=150] segments
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->segments.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->segments.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->segments.count > 150U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    for (size_t _index3_ = 0U; _index3_ < out_obj->segments.count; ++_index3_)
    {
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        {
            size_t _size_bytes3_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            const int8_t _err6_ = modrob_sensor_module_segment_params_0_1_deserialize_(
                &out_obj->segments.elements[_index3_], &buffer[offset_bits / 8U], &_size_bytes3_);
            if (_err6_ < 0)
            {
                return _err6_;
            }
            offset_bits += _size_bytes3_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);
    NUNAVUT_ASSERT(capacity_bytes >= *inout_buffer_size_bytes);

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void modrob_sensor_module_sensor_data_0_2_initialize_(modrob_sensor_module_sensor_data_0_2* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = modrob_sensor_module_sensor_data_0_2_deserialize_(out_obj, &buf, &size_bytes);
        NUNAVUT_ASSERT(err >= 0);
        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // MODROB_SENSOR_MODULE_SENSOR_DATA_0_2_INCLUDED_
