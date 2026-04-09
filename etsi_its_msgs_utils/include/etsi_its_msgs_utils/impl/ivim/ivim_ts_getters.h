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
 * @file impl/ivim/ivim_ts_getters.h
 * @brief Getter functions for the ETSI ITS IVIM (TS)
 */

#pragma once

namespace etsi_its_ivim_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_getters.h>
#include <etsi_its_msgs_utils/impl/cdd/cdd_getters_common.h>
#include <etsi_its_msgs_utils/impl/checks.h>

/**
 * @brief Retrieves the station ID from the given IVIM.
 *
 * @param ivim The IVIM from which to retrieve the station ID.
 * @return The station ID extracted from the IVIM header.
 */
inline uint32_t getStationID(const IVIM& ivim) { return getStationID(ivim.header); }

/**
 * @brief Retrieves the mandatory IVI management container from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the management container.
 * @return The IVI management container.
 */
inline IviManagementContainer getIviManagementContainer(const IVIM& ivim) { return ivim.ivi.mandatory; }

/**
 * @brief Retrieves the service provider information from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the provider.
 * @return The provider object stored in the management container.
 */
inline Provider getServiceProviderId(const IVIM& ivim) { return getIviManagementContainer(ivim).service_provider_id; }

/**
 * @brief Retrieves the encoded country code bits from a provider.
 *
 * @param provider The provider object.
 * @return The country code bit string as a boolean vector.
 */
inline std::vector<bool> getCountryCode(const Provider& provider) {
  return getBitString(provider.country_code.value, provider.country_code.bits_unused);
}

/**
 * @brief Retrieves the encoded country code bits from an IVIM.
 *
 * @param ivim The IVIM from which to retrieve the provider country code.
 * @return The country code bit string as a boolean vector.
 */
inline std::vector<bool> getCountryCode(const IVIM& ivim) { return getCountryCode(getServiceProviderId(ivim)); }

/**
 * @brief Retrieves the issuer identifier from a provider.
 *
 * @param provider The provider object.
 * @return The issuer identifier value.
 */
inline uint16_t getIssuerIdentifier(const Provider& provider) { return provider.provider_identifier.value; }

/**
 * @brief Retrieves the issuer identifier from an IVIM.
 *
 * @param ivim The IVIM from which to retrieve the issuer identifier.
 * @return The issuer identifier value.
 */
inline uint16_t getIssuerIdentifier(const IVIM& ivim) { return getIssuerIdentifier(getServiceProviderId(ivim)); }

/**
 * @brief Retrieves the IVI identification number object from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the identification number.
 * @return The IVI identification number object.
 */
inline IviIdentificationNumber getIviIdentificationNumber(const IVIM& ivim) {
  return getIviManagementContainer(ivim).ivi_identification_number;
}

/**
 * @brief Retrieves the IVI identification number value from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the identification number value.
 * @return The IVI identification number value.
 */
inline uint16_t getIviIdentificationNumberValue(const IVIM& ivim) { return getIviIdentificationNumber(ivim).value; }

/**
 * @brief Retrieves the IVI status object from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the status.
 * @return The IVI status object.
 */
inline IviStatus getIviStatus(const IVIM& ivim) { return getIviManagementContainer(ivim).ivi_status; }

/**
 * @brief Retrieves the IVI status value from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the status value.
 * @return The IVI status value.
 */
inline uint8_t getIviStatusValue(const IVIM& ivim) { return getIviStatus(ivim).value; }

/**
 * @brief Retrieves the timestamp object from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the timestamp.
 * @return The timestamp object.
 * @throws std::invalid_argument if the timestamp is not present.
 */
inline TimestampIts getTimeStamp(const IVIM& ivim) {
  const auto management = getIviManagementContainer(ivim);
  throwIfNotPresent(management.time_stamp_is_present, "ivim.ivi.mandatory.time_stamp");
  return management.time_stamp;
}

/**
 * @brief Retrieves the timestamp value from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the timestamp value.
 * @return The timestamp value in milliseconds since UTC start of 2004.
 */
inline uint64_t getTimeStampValue(const IVIM& ivim) { return getTimeStamp(ivim).value; }

/**
 * @brief Retrieves the validity start timestamp object from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the validity start timestamp.
 * @return The validity start timestamp object.
 * @throws std::invalid_argument if the timestamp is not present.
 */
inline TimestampIts getValidFrom(const IVIM& ivim) {
  const auto management = getIviManagementContainer(ivim);
  throwIfNotPresent(management.valid_from_is_present, "ivim.ivi.mandatory.valid_from");
  return management.valid_from;
}

/**
 * @brief Retrieves the validity start timestamp value from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the validity start timestamp value.
 * @return The validity start timestamp value in milliseconds since UTC start of 2004.
 */
inline uint64_t getValidFromValue(const IVIM& ivim) { return getValidFrom(ivim).value; }

/**
 * @brief Retrieves the validity end timestamp object from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the validity end timestamp.
 * @return The validity end timestamp object.
 * @throws std::invalid_argument if the timestamp is not present.
 */
inline TimestampIts getValidTo(const IVIM& ivim) {
  const auto management = getIviManagementContainer(ivim);
  throwIfNotPresent(management.valid_to_is_present, "ivim.ivi.mandatory.valid_to");
  return management.valid_to;
}

/**
 * @brief Retrieves the validity end timestamp value from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the validity end timestamp value.
 * @return The validity end timestamp value in milliseconds since UTC start of 2004.
 */
inline uint64_t getValidToValue(const IVIM& ivim) { return getValidTo(ivim).value; }

/**
 * @brief Retrieves the connected IVI structure identifiers from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the connected IVI structures.
 * @return The connected IVI structure identifier list.
 * @throws std::invalid_argument if the list is not present.
 */
inline IviIdentificationNumbers getConnectedIviStructures(const IVIM& ivim) {
  const auto management = getIviManagementContainer(ivim);
  throwIfNotPresent(management.connected_ivi_structures_is_present, "ivim.ivi.mandatory.connected_ivi_structures");
  return management.connected_ivi_structures;
}

/**
 * @brief Retrieves the optional IVI containers from the IVIM.
 *
 * @param ivim The IVIM from which to retrieve the optional containers.
 * @return The optional IVI containers.
 * @throws std::invalid_argument if the optional containers are not present.
 */
inline IviContainers getOptionalIviContainers(const IVIM& ivim) {
  throwIfNotPresent(ivim.ivi.optional_is_present, "ivim.ivi.optional");
  return ivim.ivi.optional;
}

}  // namespace etsi_its_ivim_ts_msgs::access
