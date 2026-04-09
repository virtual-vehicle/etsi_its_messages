/*
=============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2026 Virtual Vehicle Research GmbH

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=============================================================================
*/

/**
 * @file impl/ivim/ivim_ts_setters.h
 * @brief Setter functions for the ETSI ITS IVIM (TS)
 */

#pragma once

#include <etsi_its_msgs_utils/impl/constants.h>

namespace etsi_its_ivim_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_setters.h>
#include <etsi_its_msgs_utils/impl/cdd/cdd_setters_common.h>
#include <etsi_its_msgs_utils/impl/checks.h>

/**
 * @brief Sets the ITS PDU header of an IVIM.
 *
 * @param ivim The IVIM to set the ITS PDU header for.
 * @param station_id The station ID to set in the ITS PDU header.
 * @param protocol_version The protocol version to set in the ITS PDU header. Default is 0.
 */
inline void setItsPduHeader(IVIM& ivim, const uint32_t station_id, const uint8_t protocol_version = 0) {
  setStationId(ivim.header.station_id, station_id);
  throwIfOutOfRange(protocol_version, ItsPduHeader::PROTOCOL_VERSION_MIN, ItsPduHeader::PROTOCOL_VERSION_MAX,
                    "ProtocolVersion");
  ivim.header.protocol_version = protocol_version;
  ivim.header.message_id = ItsPduHeader::MESSAGE_ID_IVIM;
}

/**
 * @brief Sets the country code bit string of a provider.
 *
 * @param provider The provider object to modify.
 * @param country_code The country code bit string as a boolean vector.
 * @throws std::invalid_argument if the country code does not contain exactly 10 bits.
 */
inline void setCountryCode(Provider& provider, const std::vector<bool>& country_code) {
  if (country_code.size() != CountryCode::SIZE_BITS) {
    throw std::invalid_argument("CountryCode must contain exactly " + std::to_string(CountryCode::SIZE_BITS) +
                                " bits");
  }
  setBitString(provider.country_code, country_code);
}

/**
 * @brief Sets the issuer identifier of a provider.
 *
 * @param provider The provider object to modify.
 * @param issuer_identifier The issuer identifier value.
 */
inline void setIssuerIdentifier(Provider& provider, const uint16_t issuer_identifier) {
  throwIfOutOfRange(issuer_identifier, IssuerIdentifier::MIN, IssuerIdentifier::MAX, "IssuerIdentifier");
  provider.provider_identifier.value = issuer_identifier;
}

/**
 * @brief Sets the provider information in an IVIM management container.
 *
 * @param management_container The management container to modify.
 * @param country_code The country code bit string as a boolean vector.
 * @param issuer_identifier The issuer identifier value.
 */
inline void setServiceProviderId(IviManagementContainer& management_container, const std::vector<bool>& country_code,
                                 const uint16_t issuer_identifier) {
  setCountryCode(management_container.service_provider_id, country_code);
  setIssuerIdentifier(management_container.service_provider_id, issuer_identifier);
}

/**
 * @brief Sets the IVI identification number.
 *
 * @param management_container The management container to modify.
 * @param ivi_identification_number The IVI identification number value.
 */
inline void setIviIdentificationNumber(IviManagementContainer& management_container,
                                       const uint16_t ivi_identification_number) {
  throwIfOutOfRange(ivi_identification_number, IviIdentificationNumber::MIN, IviIdentificationNumber::MAX,
                    "IviIdentificationNumber");
  management_container.ivi_identification_number.value = ivi_identification_number;
}

/**
 * @brief Sets the IVI identification number in an IVIM.
 *
 * @param ivim The IVIM to modify.
 * @param ivi_identification_number The IVI identification number value.
 */
inline void setIviIdentificationNumber(IVIM& ivim, const uint16_t ivi_identification_number) {
  setIviIdentificationNumber(ivim.ivi.mandatory, ivi_identification_number);
}

/**
 * @brief Sets the IVI status.
 *
 * @param management_container The management container to modify.
 * @param status The IVI status value.
 */
inline void setIviStatus(IviManagementContainer& management_container, const uint8_t status) {
  throwIfOutOfRange(status, IviStatus::MIN, IviStatus::MAX, "IviStatus");
  management_container.ivi_status.value = status;
}

/**
 * @brief Sets the IVI status in an IVIM.
 *
 * @param ivim The IVIM to modify.
 * @param status The IVI status value.
 */
inline void setIviStatus(IVIM& ivim, const uint8_t status) { setIviStatus(ivim.ivi.mandatory, status); }

/**
 * @brief Sets the timestamp in an IVIM management container.
 *
 * @param management_container The management container to modify.
 * @param unix_nanosecs The Unix timestamp in nanoseconds including leap seconds handling.
 * @param n_leap_seconds The number of leap seconds since 2004.
 */
inline void setTimeStamp(
    IviManagementContainer& management_container, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  setTimestampITS(management_container.time_stamp, unix_nanosecs, n_leap_seconds);
  management_container.time_stamp_is_present = true;
}

/**
 * @brief Sets the timestamp in an IVIM.
 *
 * @param ivim The IVIM to modify.
 * @param unix_nanosecs The Unix timestamp in nanoseconds including leap seconds handling.
 * @param n_leap_seconds The number of leap seconds since 2004.
 */
inline void setTimeStamp(
    IVIM& ivim, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  setTimeStamp(ivim.ivi.mandatory, unix_nanosecs, n_leap_seconds);
}

/**
 * @brief Sets the validity start timestamp in an IVIM management container.
 *
 * @param management_container The management container to modify.
 * @param unix_nanosecs The Unix timestamp in nanoseconds including leap seconds handling.
 * @param n_leap_seconds The number of leap seconds since 2004.
 */
inline void setValidFrom(
    IviManagementContainer& management_container, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  setTimestampITS(management_container.valid_from, unix_nanosecs, n_leap_seconds);
  management_container.valid_from_is_present = true;
}

/**
 * @brief Sets the validity start timestamp in an IVIM.
 *
 * @param ivim The IVIM to modify.
 * @param unix_nanosecs The Unix timestamp in nanoseconds including leap seconds handling.
 * @param n_leap_seconds The number of leap seconds since 2004.
 */
inline void setValidFrom(
    IVIM& ivim, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  setValidFrom(ivim.ivi.mandatory, unix_nanosecs, n_leap_seconds);
}

/**
 * @brief Sets the validity end timestamp in an IVIM management container.
 *
 * @param management_container The management container to modify.
 * @param unix_nanosecs The Unix timestamp in nanoseconds including leap seconds handling.
 * @param n_leap_seconds The number of leap seconds since 2004.
 */
inline void setValidTo(
    IviManagementContainer& management_container, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  setTimestampITS(management_container.valid_to, unix_nanosecs, n_leap_seconds);
  management_container.valid_to_is_present = true;
}

/**
 * @brief Sets the validity end timestamp in an IVIM.
 *
 * @param ivim The IVIM to modify.
 * @param unix_nanosecs The Unix timestamp in nanoseconds including leap seconds handling.
 * @param n_leap_seconds The number of leap seconds since 2004.
 */
inline void setValidTo(
    IVIM& ivim, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  setValidTo(ivim.ivi.mandatory, unix_nanosecs, n_leap_seconds);
}

/**
 * @brief Sets the optional IVI containers in an IVIM.
 *
 * @param ivim The IVIM to modify.
 * @param containers The optional IVI containers.
 */
inline void setOptionalIviContainers(IVIM& ivim, const IviContainers& containers) {
  ivim.ivi.optional = containers;
  ivim.ivi.optional_is_present = true;
}

}  // namespace etsi_its_ivim_ts_msgs::access
