#pragma once

#include <memory>
#include <sstream>
#include <tinyxml2.h>
#include <calibu/Platform.h>
#include <calibu/pcalib/pcalib.h>

namespace calibu
{

/**
 * Reads photometric calibration objects from an XML file
 */
class PhotoCalibReader
{
  public:

    /**
     * Creates PhotoCalibReader object for reading from the given file
     * @param filename full file path to the input XML file
     */
    PhotoCalibReader(const std::string& filename);

    /**
     * Reads calibration from file and stores results in the give argument
     * @param calib output calibration object
     */
    void Read(PhotoCalibd& calib);

  protected:

    /**
     * Prepares XML file for parsing
     */
    void PrepareRead();

    /**
     * Reads all responses models and writes output to given calibration
     * @param calib output calibration object
     */
    void ReadResponses(PhotoCalibd& calib);

    /**
     * Reads all vignetting models and writes output to given calibration
     * @param calib output calibration object
     */
    void ReadVignettings(PhotoCalibd& calib);

    /**
     * Reads a response model from the given XML element.
     * @param element XML element to be parsed
     * @return parsed response model
     */
    std::shared_ptr<Response<double>> ReadResponse(
        const tinyxml2::XMLElement* element);

    /**
     * Reads a vignetting model from the given XML element.
     * @param element XML element to be parsed
     * @return parsed vignetting model
     */
    std::shared_ptr<Vignetting<double>> ReadVignetting(
        const tinyxml2::XMLElement* element);

    /**
     * Clears the read XML document data
     */
    void FinishRead();

    /**
     * Create default response of specified type
     * @param type string specifying the desired response type
     * @return default response model of given type
     */
    static std::shared_ptr<Response<double>> CreateResponse(
        const std::string& type);

    /**
     * Create default vignetting of specified type
     * @param type string specifying the desired vignetting type
     * @param width width of the vignetting model
     * @param height height of the vignetting model
     * @return default vignetting model of given type
     */
    static std::shared_ptr<Vignetting<double> > CreateVignetting(
        const std::string& type, int width, int height);

    /**
     * Builds a matrix from the given string. The expected dimension of the
     * matrix is read from the given matrix. If the matrix is significantly
     * small, the text will be interpretted as plain text, otherwise it will
     * be decoded as a base-64 string.
     * @param text plain text encoding the matrix values
     * @param matrix input/output matrix
     */
    static void GetMatrix(const char* text, Eigen::MatrixXd& matrix);

    /**
     * Builds a matrix from the given plain text string. The expected dimension
     * of the matrix is read from the given matrix
     * @param text text string of the matrix values
     * @param matrix input/output matrix
     */
    static void GetShortMatrix(const char* text, Eigen::MatrixXd& matrix);

    /**
     * Builds a matrix from the given base-64 encoded string.
     * The expected dimension of the matrix is read from the given matrix
     * @param text base-64 encoded string of the matrix values
     * @param matrix input/output matrix
     */
    static void GetLongMatrix(const char* text, Eigen::MatrixXd& matrix);

  protected:

    /** Full file path to the input XML file */
    std::string filename_;

    /** XML document read from file */
    tinyxml2::XMLDocument document_;

    /** Root element node in the read XML file */
    const tinyxml2::XMLElement* root_;
};

/**
 * Writes photometric calibration objects to an XML file
 */
class PhotoCalibWriter
{
  public:

    /**
     * Creates PhotoCalibWriter object for writting to the given file
     * @param filename full file path to the output XML file
     */
    PhotoCalibWriter(const std::string& filename);

    /**
     * Writes a calibration from file and stores results in the give argument
     * @param calib output calibration object
     */
    void Write(const PhotoCalibd& calib);

  protected:

    /**
     * Prepares XML file for writing
     */
    void PrepareWrite();

    /**
     * Writes all the response models in the given calibration to file
     * @param calib calibration containing responses to be written
     */
    void WriteResponses(const PhotoCalibd& calib);

    /**
     * Writes all the vignetting models in the given calibration to file
     * @param calib calibration containing vignettings to be written
     */
    void WriteVignettings(const PhotoCalibd& calib);

    /**
     * Writes the given response to file
     * @param response response to be written
     */
    void WriteResponse(const Response<double>& response);

    /**
     * Writes the given vignetting to file
     * @param vignetting vignetting to be written
     */
    void WriteVignetting(const Vignetting<double>& vignetting);

    /**
     * Closes the XML document
     */
    void FinishWrite();

    /**
     * Builds a string from the given matrix. If the given matrix is
     * sufficiently small, it will simply be converted plain text. Otherwise it
     * will be encoded into a base-64 string.
     * @param matrix
     * @return
     */
    static std::string GetText(const Eigen::MatrixXd& matrix);

    /**
     * Creates a plain text string for the given matrix
     * @param matrix matrix to be converted
     * @return plain text string of matrix values
     */
    static std::string GetShortText(const Eigen::MatrixXd& matrix);

    /**
     * Builds a base-64 encoded string from the given matrix
     * @param matrix matrix to be converted
     * @return encoded string
     */
    static std::string GetLongText(const Eigen::MatrixXd& matrix);

  protected:

    /** Full file path to the output XML file */
    std::string filename_;

    /** Intermediate XML document to be constructed */
    tinyxml2::XMLDocument document_;
};

/**
 * Parses the photometric calibration from the given XML file
 * @param filename XML file to be parsed
 * @return photometric calibration parse from the XML file
 */
CALIBU_EXPORT
std::shared_ptr<PhotoCalibd> ReadXmlPhotoCalib(const std::string& filename);

/**
 * Writes the given photometric calibration to the specified XML file
 * @param filename output XML file to be written
 * @param calib calibration object to be written
 */
CALIBU_EXPORT
void WriteXmlPhotoCalib(const std::string& filename, const PhotoCalibd& calib);

} // namespace calibu