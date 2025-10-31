#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pyro_core_dma_heap.h"

#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( xHeapStructSize << 1 ) )

#define heapBITS_PER_BYTE		( ( size_t ) 8 )

typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
    size_t xBlockSize;						/*<< The size of the free block. */
} BlockLink_t;

static const size_t xHeapStructSize	= ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

/* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
member of an BlockLink_t structure is set then the block belongs to the
application.  When the bit is free the block is still part of the free heap
space. */
static size_t xBlockAllocatedBit = 0;


/* ========== DMA heap extension ========== */
/*
 * 新增一个独立的 DMA 堆。它与主堆完全独立，使用相同的分配策略（合并空闲块）。
 *
 * 配置选项（可在 FreeRTOSConfig.h 中定义）:
 *  - configTOTAL_DMA_HEAP_SIZE           : DMA 堆大小（字节）。若未定义或为 0，则 DMA 堆不可用，相关函数返回 NULL / 0。
 *  - configAPPLICATION_ALLOCATED_DMA_HEAP: 若为 1，应用需提供 ucDmaHeap[] 缓冲区（同主堆的配置方式）。
 *
 * 提供函数:
 *  - void *pvPortDmaMalloc( size_t xWantedSize );
 *  - void  vPortDmaFree( void *pv );
 *  - void  vPortGetDmaHeapStats( HeapStats_t *pxHeapStats );
 *
 * 注意：该实现尽量与主堆行为一致（对齐、trace、assert），但不会改变原有主堆逻辑。
 */

#ifndef configTOTAL_DMA_HEAP_SIZE
	/* 默认 DMA 堆大小为 0（不可用），如果需要使用请在 FreeRTOSConfig.h 中定义一个合理值 */
	#define configTOTAL_DMA_HEAP_SIZE 1024
#endif

#if( configTOTAL_DMA_HEAP_SIZE > 0 )

	#if( configAPPLICATION_ALLOCATED_DMA_HEAP == 1 )
		extern uint8_t ucDmaHeap[ configTOTAL_DMA_HEAP_SIZE ];
	#else
		__attribute__((section(".dma_heap"))) static uint8_t ucDmaHeap[ configTOTAL_DMA_HEAP_SIZE ];
	#endif /* configAPPLICATION_ALLOCATED_DMA_HEAP */

	/* DMA 堆的起始/结束标记和统计变量（与主堆分离） */
	static BlockLink_t xDmaStart, *pxDmaEnd = NULL;

	static size_t xDmaFreeBytesRemaining = 0U;
	static size_t xDmaMinimumEverFreeBytesRemaining = 0U;
	static size_t xDmaNumberOfSuccessfulAllocations = 0;
	static size_t xDmaNumberOfSuccessfulFrees = 0;

	/* 用于区分已分配块的标志（与主堆共享 same allocated bit） */
	/* xBlockAllocatedBit 已在上面计算并初始化。 */

	/* DMA 堆函数原型 */
	static void prvDmaHeapInit( void );
	static void prvInsertDmaBlockIntoFreeList( BlockLink_t *pxBlockToInsert );

	void *pvPortDmaMalloc( size_t xWantedSize )
	{
	BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
	void *pvReturn = NULL;

		/* 如果 DMA 堆大小为 0，直接返回 NULL */
		if( configTOTAL_DMA_HEAP_SIZE == 0 )
		{
			return NULL;
		}

		vTaskSuspendAll();
		{
			/* 初始化 DMA 堆（首次调用时） */
			if( pxDmaEnd == NULL )
			{
				prvDmaHeapInit();
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			/* 与主堆相同的检查/对齐逻辑 */
			if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
			{
				if( xWantedSize > 0 )
				{
					xWantedSize += xHeapStructSize;

					if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
					{
						xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
						configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}

				if( ( xWantedSize > 0 ) && ( xWantedSize <= xDmaFreeBytesRemaining ) )
				{
					/* 在 DMA 空闲链表中查找合适块（从低地址向高地址） */
					pxPreviousBlock = &xDmaStart;
					pxBlock = xDmaStart.pxNextFreeBlock;
					while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
					{
						pxPreviousBlock = pxBlock;
						pxBlock = pxBlock->pxNextFreeBlock;
					}

					if( pxBlock != pxDmaEnd )
					{
						pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );

						/* 从空闲链表中摘除该块 */
						pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

						/* 如果块过大，则拆分 */
						if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
						{
							pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
							configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

							pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
							pxBlock->xBlockSize = xWantedSize;

							prvInsertDmaBlockIntoFreeList( pxNewBlockLink );
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}

						xDmaFreeBytesRemaining -= pxBlock->xBlockSize;

						if( xDmaFreeBytesRemaining < xDmaMinimumEverFreeBytesRemaining )
						{
							xDmaMinimumEverFreeBytesRemaining = xDmaFreeBytesRemaining;
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}

						pxBlock->xBlockSize |= xBlockAllocatedBit;
						pxBlock->pxNextFreeBlock = NULL;
						xDmaNumberOfSuccessfulAllocations++;
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			traceMALLOC( pvReturn, xWantedSize );
		}
		( void ) xTaskResumeAll();

		#if( configUSE_MALLOC_FAILED_HOOK == 1 )
		{
			if( pvReturn == NULL )
			{
				extern void vApplicationMallocFailedHook( void );
				vApplicationMallocFailedHook();
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		#endif

		configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
		return pvReturn;
	}

	void vPortDmaFree( void *pv )
	{
	uint8_t *puc = ( uint8_t * ) pv;
	BlockLink_t *pxLink;

		if( pv != NULL )
		{
			/* DMA 堆未启用则直接返回（不应发生） */
			if( configTOTAL_DMA_HEAP_SIZE == 0 )
			{
				return;
			}

			/* The memory being freed will have an BlockLink_t structure immediately
			before it. */
			puc -= xHeapStructSize;

			/* This casting is to keep the compiler from issuing warnings. */
			pxLink = ( void * ) puc;

			/* Check the block is actually allocated. */
			configASSERT( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 );
			configASSERT( pxLink->pxNextFreeBlock == NULL );

			if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
			{
				if( pxLink->pxNextFreeBlock == NULL )
				{
					/* The block is being returned to the heap - it is no longer
					allocated. */
					pxLink->xBlockSize &= ~xBlockAllocatedBit;

					vTaskSuspendAll();
					{
						/* Add this block to the list of free blocks. */
						xDmaFreeBytesRemaining += pxLink->xBlockSize;
						traceFREE( pv, pxLink->xBlockSize );
						prvInsertDmaBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
						xDmaNumberOfSuccessfulFrees++;
					}
					( void ) xTaskResumeAll();
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
	}

	static void prvDmaHeapInit( void )
	{
	BlockLink_t *pxFirstFreeBlock;
	uint8_t *pucAlignedHeap;
	size_t uxAddress;
	size_t xTotalHeapSize = configTOTAL_DMA_HEAP_SIZE;

		/* Ensure the heap starts on a correctly aligned boundary. */
		uxAddress = ( size_t ) ucDmaHeap;

		if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
		{
			uxAddress += ( portBYTE_ALIGNMENT - 1 );
			uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
			xTotalHeapSize -= uxAddress - ( size_t ) ucDmaHeap;
		}

		pucAlignedHeap = ( uint8_t * ) uxAddress;

		/* xDmaStart holds pointer to first item in free list. */
		xDmaStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
		xDmaStart.xBlockSize = ( size_t ) 0;

		/* pxDmaEnd marks the end of the DMA heap */
		uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
		uxAddress -= xHeapStructSize;
		uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
		pxDmaEnd = ( void * ) uxAddress;
		pxDmaEnd->xBlockSize = 0;
		pxDmaEnd->pxNextFreeBlock = NULL;

		/* Single free block covering the whole DMA heap (minus pxDmaEnd) */
		pxFirstFreeBlock = ( void * ) pucAlignedHeap;
		pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
		pxFirstFreeBlock->pxNextFreeBlock = pxDmaEnd;

		xDmaMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
		xDmaFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

		/* Note: xBlockAllocatedBit is shared with main heap and already set in prvHeapInit when first main heap init occurs.
		 * If main heap has not been initialized yet, ensure xBlockAllocatedBit is set:
		 */
		if( xBlockAllocatedBit == 0 )
		{
			xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );
		}
	}

static void prvInsertDmaBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
	{
	    BlockLink_t *pxIterator;
	    uint8_t *puc;

	    /* Iterate through the list until a block is found that has a higher address
            than the block being inserted. */
	    for( pxIterator = &xDmaStart;
                 pxIterator->pxNextFreeBlock < pxBlockToInsert &&
                 pxIterator->pxNextFreeBlock != NULL;
                 pxIterator = pxIterator->pxNextFreeBlock )
	    {
	        /* Nothing to do here. */
	    }

	    /* Merge with previous block if adjacent. */
	    puc = ( uint8_t * ) pxIterator;
	    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	    {
	        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
	        pxBlockToInsert = pxIterator;
	    }

	    /* Merge with next block if adjacent. */
	    puc = ( uint8_t * ) pxBlockToInsert;
	    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	    {
	        if( pxIterator->pxNextFreeBlock != pxDmaEnd )
	        {
	            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
	            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
	        }
	        else
	        {
	            pxBlockToInsert->pxNextFreeBlock = pxDmaEnd;
	        }
	    }
	    else
	    {
	        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	    }

	    if( pxIterator != pxBlockToInsert )
	    {
	        pxIterator->pxNextFreeBlock = pxBlockToInsert;
	    }
	}


	void vPortGetDmaHeapStats( HeapStats_t *pxHeapStats )
	{
	BlockLink_t *pxBlock;
	size_t xBlocks = 0, xMaxSize = 0, xMinSize = portMAX_DELAY;

		/* If DMA heap disabled, zero stats */
		if( configTOTAL_DMA_HEAP_SIZE == 0 || pxDmaEnd == NULL )
		{
			pxHeapStats->xSizeOfLargestFreeBlockInBytes = 0;
			pxHeapStats->xSizeOfSmallestFreeBlockInBytes = 0;
			pxHeapStats->xNumberOfFreeBlocks = 0;
			pxHeapStats->xAvailableHeapSpaceInBytes = 0;
			pxHeapStats->xNumberOfSuccessfulAllocations = 0;
			pxHeapStats->xNumberOfSuccessfulFrees = 0;
			pxHeapStats->xMinimumEverFreeBytesRemaining = 0;
			return;
		}

		vTaskSuspendAll();
		{
			pxBlock = xDmaStart.pxNextFreeBlock;

			if( pxBlock != NULL )
			{
				do
				{
					xBlocks++;

					if( pxBlock->xBlockSize > xMaxSize )
					{
						xMaxSize = pxBlock->xBlockSize;
					}

					if( pxBlock->xBlockSize < xMinSize )
					{
						xMinSize = pxBlock->xBlockSize;
					}

					pxBlock = pxBlock->pxNextFreeBlock;
				} while( pxBlock != pxDmaEnd );
			}
		}
		xTaskResumeAll();

		pxHeapStats->xSizeOfLargestFreeBlockInBytes = xMaxSize;
		pxHeapStats->xSizeOfSmallestFreeBlockInBytes = xMinSize;
		pxHeapStats->xNumberOfFreeBlocks = xBlocks;

		taskENTER_CRITICAL();
		{
			pxHeapStats->xAvailableHeapSpaceInBytes = xDmaFreeBytesRemaining;
			pxHeapStats->xNumberOfSuccessfulAllocations = xDmaNumberOfSuccessfulAllocations;
			pxHeapStats->xNumberOfSuccessfulFrees = xDmaNumberOfSuccessfulFrees;
			pxHeapStats->xMinimumEverFreeBytesRemaining = xDmaMinimumEverFreeBytesRemaining;
		}
		taskEXIT_CRITICAL();
	}

#else /* configTOTAL_DMA_HEAP_SIZE == 0 */

/* 如果未启用 DMA 堆，则提供弱的替代实现，方便上层调用不必 #ifdef */
void *pvPortDmaMalloc( size_t xWantedSize )
{
	return malloc(xWantedSize);
}

void vPortDmaFree( void *pv )
{
        free(pv);
}

void vPortGetDmaHeapStats( HeapStats_t *pxHeapStats )
{
	if( pxHeapStats != NULL )
	{
		pxHeapStats->xSizeOfLargestFreeBlockInBytes = 0;
		pxHeapStats->xSizeOfSmallestFreeBlockInBytes = 0;
		pxHeapStats->xNumberOfFreeBlocks = 0;
		pxHeapStats->xAvailableHeapSpaceInBytes = 0;
		pxHeapStats->xNumberOfSuccessfulAllocations = 0;
		pxHeapStats->xNumberOfSuccessfulFrees = 0;
		pxHeapStats->xMinimumEverFreeBytesRemaining = 0;
	}
}

#endif /* configTOTAL_DMA_HEAP_SIZE */

