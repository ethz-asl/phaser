#include "packlo/common/data/file-system-helper.h"

#include <boost/filesystem.hpp>


namespace data {

void FileSystemHelper::readDirectory(const std::string& directory,
		std::vector<std::string>* files) {
  boost::filesystem::path p(directory);                                         
   boost::filesystem::directory_iterator start(p);                               
   boost::filesystem::directory_iterator end;                                    
   std::transform(start, end, std::back_inserter(*files),                        
       [] (const boost::filesystem::directory_entry& entry) {                    
         return entry.path().leaf().string();                                    
     });   
}

} // namespace data
