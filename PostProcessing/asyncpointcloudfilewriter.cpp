#include "asyncpointcloudfilewriter.h"

AsyncPointCloudFileWriter::AsyncPointCloudFileWriter(const QString& fileName)
{
    this->fileName = fileName;
    fileHandleMutex.lock();     // This mutex is used to synchronize isOpenedSuccessfully, therefore lock it already
}

AsyncPointCloudFileWriter::~AsyncPointCloudFileWriter()
{
    terminateRequest = true;
    wait();
}

bool AsyncPointCloudFileWriter::isOpenedSuccessfully(void)
{
    fileHandleMutex.lock();
    bool retval = fileOpenedSuccesfully;
    fileHandleMutex.unlock();
    return retval;
}

void AsyncPointCloudFileWriter::run()
{
    file = new QFile(fileName);
    if (!file->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        fileOpenedSuccesfully = false;
        fileHandleMutex.unlock(); // Allow reading of the opened state
        return;
    }

    textStream = new QTextStream(file);

    fileOpenedSuccesfully = true;

    fileHandleMutex.unlock(); // Allow reading of the opened state

    while (!terminateRequest)
    {
        waitConditionMutex.lock();
        waitCondition.wait(&waitConditionMutex, 100);

        outBufferMutex.lock();
        while (outBuffer.contains(nextChunkToWrite))
        {
            PointCloudGeneratorLidarThread::Output outputData = outBuffer.take(nextChunkToWrite);
            outBufferMutex.unlock();

            for (auto item : *outputData.points.get())
            {
                QString lineOut = QString::number(item.hitPoint.x(), 'f', 4) +
                          "\t" + QString::number(item.hitPoint.y(), 'f', 4) +
                          "\t" + QString::number(item.hitPoint.z(), 'f', 4) +
                          "\t" + QString::number(item.normal.x(), 'f', 4) +
                          "\t" + QString::number(item.normal.y(), 'f', 4) +
                          "\t" + QString::number(item.normal.z(), 'f', 4);

                textStream->operator<<(lineOut + "\n");
            }


            outBufferMutex.lock();
            nextChunkToWrite++;
        }
        outBufferMutex.unlock();

        waitConditionMutex.unlock();
    }

    textStream->flush();
    delete textStream;

    file->close();
    delete file;
}

void AsyncPointCloudFileWriter::addPoints(const PointCloudGeneratorLidarThread::Output& out)
{
    outBufferMutex.lock();
    outBuffer.insert(out.workUnit.chunkIndex, out);
    outBufferMutex.unlock();

    waitCondition.wakeOne();
}
