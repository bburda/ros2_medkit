Scripts
=======

.. req:: POST /{entity}/scripts
   :id: REQ_INTEROP_040
   :status: verified
   :tags: Scripts

   The endpoint shall upload a new diagnostic script or test script to the addressed entity.

.. req:: GET /{entity}/scripts
   :id: REQ_INTEROP_041
   :status: verified
   :tags: Scripts

   The endpoint shall list all scripts stored on the addressed entity.

.. req:: GET /{entity}/scripts/{id}
   :id: REQ_INTEROP_042
   :status: verified
   :tags: Scripts

   The endpoint shall return metadata and content information for the addressed script.

.. req:: DELETE /{entity}/scripts/{id}
   :id: REQ_INTEROP_043
   :status: verified
   :tags: Scripts

   The endpoint shall delete the addressed script from the entity.

.. req:: POST /{entity}/scripts/{id}/executions
   :id: REQ_INTEROP_044
   :status: verified
   :tags: Scripts

   The endpoint shall start an execution of the addressed script on the entity.

.. req:: GET /{entity}/scripts/{id}/executions/{exec-id}
   :id: REQ_INTEROP_046
   :status: verified
   :tags: Scripts

   The endpoint shall return status and results for the addressed script execution.

.. req:: PUT /{entity}/scripts/{id}/executions/{exec-id}
   :id: REQ_INTEROP_047
   :status: verified
   :tags: Scripts

   The endpoint shall control or modify the addressed script execution, if supported.
