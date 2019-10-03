/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <string.h>

#include "mcuboot_config/mcuboot_config.h"

#ifdef MCUBOOT_SIGN_RSA
#include "bootutil/sign_key.h"
#include "bootutil/sha256.h"

#include "mbedtls/rsa.h"
#include "mbedtls/asn1.h"
#include "mbedtls/version.h"

#include "bootutil_priv.h"

/*
 * Parse the public key used for signing. Simple RSA format.
 */
static int
bootutil_parse_rsakey(mbedtls_rsa_context *ctx, uint8_t **p, uint8_t *end)
{
    int rc;
    size_t len;

    if ((rc = mbedtls_asn1_get_tag(p, end, &len,
          MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE)) != 0) {
        return -1;
    }

    if (*p + len != end) {
        return -2;
    }

    if ((rc = mbedtls_asn1_get_mpi(p, end, &ctx->N)) != 0 ||
      (rc = mbedtls_asn1_get_mpi(p, end, &ctx->E)) != 0) {
        return -3;
    }

    ctx->len = mbedtls_mpi_size(&ctx->N);

    if (*p != end) {
        return -4;
    }

    /* The mbedtls version is more than 2.6.1 */
#if MBEDTLS_VERSION_NUMBER > 0x02060100
    rc = mbedtls_rsa_import(ctx, &ctx->N, NULL, NULL, NULL, &ctx->E);
    if (rc != 0) {
        return -5;
    }
#endif

    rc = mbedtls_rsa_check_pubkey(ctx);
    if (rc != 0) {
        return -6;
    }

    ctx->len = mbedtls_mpi_size(&ctx->N);

    return 0;
}

int
bootutil_verify_sig(uint8_t *hash, uint32_t hlen, uint8_t *sig, size_t slen,
  uint8_t key_id)
{
    mbedtls_rsa_context ctx;
    int rc;
    uint8_t *cp;
    uint8_t *end;

    mbedtls_rsa_init(&ctx, 0, 0);

    cp = (uint8_t *)bootutil_keys[key_id].key;
    end = cp + *bootutil_keys[key_id].len;

    rc = bootutil_parse_rsakey(&ctx, &cp, end);
    if (rc || slen != ctx.len) {
        mbedtls_rsa_free(&ctx);
        return rc;
    }
    rc = mbedtls_rsa_rsassa_pss_verify_ext(&ctx, NULL, NULL,
            MBEDTLS_RSA_PUBLIC, MBEDTLS_MD_SHA256, hlen, hash,
            MBEDTLS_MD_SHA256, MBEDTLS_RSA_SALT_LEN_ANY, sig);
    mbedtls_rsa_free(&ctx);

    return rc;
}
#endif /* MCUBOOT_SIGN_RSA */
