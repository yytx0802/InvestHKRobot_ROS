#ifndef PACKET_FINDER_HPP_
#define PACKET_FINDER_HPP_
#include <ecl/containers.hpp>
#include <ecl/sigslots.hpp>
#include <iomanip>

namespace wit {

class PacketFinderBase {
 public:
  typedef ecl::PushAndPop<unsigned char> BufferType;

  enum packetFinderState {
    clearBuffer = 0,
    waitingForStx,
    waitingForFlag,
    waitingForPayloadSize,
    waitingForPayloadToEtx,
    waitingForEtx,
  };
  enum packetFinderState state;

 protected:
  unsigned int size_stx;
  unsigned int size_etx;
  unsigned int size_length_field;
  bool variable_size_payload;
  unsigned int size_max_payload;
  unsigned int size_payload;
  unsigned int size_checksum_field;

  BufferType STX;
  BufferType ETX;
  BufferType buffer;

  bool verbose;

  ecl::Signal<const std::string &> sig_warn, sig_error;

 public:
  PacketFinderBase(); /**< Default constructor. Use with configure(). **/

  virtual ~PacketFinderBase(){};

  void configure(const std::string &sigslots_namespace,
                 const BufferType &putStx, const BufferType &putEtx,
                 unsigned int sizeLengthField, unsigned int sizeMaxPayload,
                 unsigned int sizeChecksumField, bool variableSizePayload);
  void clear();
  void enableVerbose();
  virtual bool update(const unsigned char *incoming,
                      unsigned int numberOfIncoming);
  virtual bool checkSum();
  unsigned int numberOfDataToRead();
  void getBuffer(BufferType &bufferRef);
  void getPayload(BufferType &bufferRef);

 protected:
  bool WaitForStx(const unsigned char datum);
  bool WaitForFlag(const unsigned char datum);
  bool waitForPayloadSize(const unsigned char *incoming,
                          unsigned int numberOfIncoming);
  bool waitForEtx(const unsigned char incoming, bool &foundPacket);
  bool waitForPayloadAndEtx(const unsigned char *incoming,
                            unsigned int numberOfIncoming, bool &foundPacket);
};
};

#endif /* PACKET_FINDER_HPP_ */
