/**
 * @file  pos_mm.h  
 * @brief Memory mapping and virual memory management
 * @author Runby F.
 * @date 2022-3-09
 * @copyright Polysense
 */

#ifndef __POS_MM_H__
#define __POS_MM_H__

/**
 * @brief Memory mapping and management structure
 */
typedef struct {
  /**
  * @brief Virtual address mapping to physical address
  * @param[in] hdr Physical header address storing these virtual content
  * @param[in] addr Virtual address to be mapped
  * @return Mapped physical address
  */
  pos_u32_t (*map_addr)(void *hdr, pos_u32_t addr) ;

  /**
  * @brief Virtual address table mapping to physical address table
  * @param[in] hdr Physical header address storing these virtual content
  * @param[in] virtual Virtual address table to be mapped
  * @param[out] physical Final physical address table after mapping
  * @param[in] len Number of bytes to be mapped (must be 4/8/12/16/20/.../4*n)
  */
  void (*mmap)(void *hdr, void *virtual, void *physical, pos_size_t len);

  /**
  * @brief Translate a physical header from virtual to physical address
  * @param[in] hdr Physical header address storing these virtual content
  * @param[out] export Final header address storing the mapped physical content
  * @param[in] len Number of bytes to be mapped (must be 4/8/12/16/20/.../4*n)
  * @param[out] blk_len The actual virtual block length of this giving hdr
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */  
  pos_status_t (*hdr2exp)(void *hdr, void *export, pos_size_t len, pos_size_t *blk_len);

} pos_lib_mm_t;

#endif
