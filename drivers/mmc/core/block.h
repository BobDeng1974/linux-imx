#ifndef _MMC_CORE_BLOCK_H
#define _MMC_CORE_BLOCK_H

struct mmc_queue;
struct request;

void mmc_blk_issue_rq(struct mmc_queue *mq, struct request *req);

enum mmc_issued;

enum mmc_issued mmc_blk_cqe_issue_rq(struct mmc_queue *mq,
				     struct request *req);
void mmc_blk_cqe_complete_rq(struct request *rq);
void mmc_blk_cqe_recovery(struct mmc_queue *mq);

#endif
