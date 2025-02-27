/*
 * Amazon FreeRTOS V1.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

#ifndef AWS_INTEGRATION_TEST_TCP_CONFIG_H
#define AWS_INTEGRATION_TEST_TCP_CONFIG_H

/**
 * @file aws_integration_test_tcp_portable.h
 * @brief Port-specific variables for TCP tests. */

/**
 * @brief The number of sockets that can be open at one time on a port.
 *
 */
#ifdef MEMP_NUM_NETCONN
#define integrationtestportableMAX_NUM_UNSECURE_SOCKETS (MEMP_NUM_NETCONN)
#else
#define integrationtestportableMAX_NUM_UNSECURE_SOCKETS 8
#endif

/**
 * @brief Indicates how much longer than the specified timeout is acceptable for
 * RCVTIMEO tests.
 *
 * This value can be used to compensate for clock differences, and other
 * code overhead.
 */
#define integrationtestportableTIMEOUT_OVER_TOLERANCE 20

/**
 * @brief Indicates how much less time than the specified timeout is acceptable for
 * RCVTIMEO tests.
 *
 * This value must be 0 unless networking is performs on a separate processor.
 * If networking and tests are on different CPUs, an "under tolerance" is acceptable.
 * For tests where same clock is used for networking and tests.
 */
#define integrationtestportableTIMEOUT_UNDER_TOLERANCE 10

/**
 * @brief Indicates how long  receive needs to wait for data before Timeout happens.
 *
 */
#define integrationtestportableRECEIVE_TIMEOUT 30000

/**
 * @brief Indicates how long  send needs to wait before Timeout happens.
 *
 */
#define integrationtestportableSEND_TIMEOUT 30000

#define tcptestECHO_TEST_RXTX_TIMEOUT 30000

#define tcptestECHO_TEST_SYNC_TIMEOUT 240000

#endif /*AWS_INTEGRATION_TEST_TCP_CONFIG_H */
